/*******************************************************************************
 * Copyright (c) 2018, Thomas Stastny, Autonomous Systems Lab (ASL), ETH Zurich,
 * Switzerland
 *
 * You can contact the author at <thomas.stastny@mavt.ethz.ch>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3) Neither the name of ETHZ-ASL nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ros/ros.h>
#include <ros/package.h>

#include <fw_nmpc.h>

#include <math.h>
#include <string.h>

#include <fw_ctrl/AcadoVars.h>
#include <fw_ctrl/NmpcInfo.h>

using namespace fw_nmpc;

FwNmpc::FwNmpc() :
		x0_(acadoVariables.x0),
		x_(acadoVariables.x),
		u_(acadoVariables.u),
		y_(acadoVariables.y),
		yN_(acadoVariables.yN),
		W_(acadoVariables.W),
		od_(acadoVariables.od, IDX_OD_TERR_DATA),
		terr_data_(acadoVariables.od + IDX_OD_TERR_DATA, ACADO_NOD-IDX_OD_TERR_DATA),
		WN_(acadoVariables.x0),
		loop_rate_(10.0),
		t_step_(0.1),
{
	ROS_INFO("Instance of NMPC created");

	/* subscribers */
	imu_sub_ = nmpc_.subscribe("/mavros/imu/data", 1, &FwNmpc::imuCb, this);
	grid_map_sub_ = nmpc_.subscribe("/fw_lander/global_map", 1, &FwNmpc::gridMapCb, this);
	home_pos_sub_ = nmpc_.subscribe("/mavros/home_position", 1, &FwNmpc::homePosCb, this);
	local_pos_sub_ = nmpc_.subscribe("/mavros/local_position/pose", 1, &FwNmpc::localPosCb, this);
  local_vel_sub_ = nmpc_.subscribe("/mavros/local_position/velocity", 1, &FwNmpc::localVelCb, this);
	sys_status_sub_ = nmpc_.subscribe("/mavros/state", 1, &FwNmpc::sysStatusCb, this);
	wind_est_sub_ = nmpc_.subscribe("/mavros/wind_estimation", 1, &FwNmpc::windEstCb, this);

	/* publishers */
	att_sp_pub_ = nmpc_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1);
	obctrl_status_pub_ = nmpc_.advertise<std_msgs::Int32>("/nmpc/status", 1);
	thrust_pub_ = nmpc_.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/thrust", 1);
}

/* callbacks */

void FwNmpc::imuCb(const sensor_msgs::Imu::ConstPtr& msg) // IMU msg callback
{
	Eigen::Quaterniond q;
	tf::quaternionMsgToEigen(msg->orientation, q);
	x0_euler_ = q.toRotationMatrix().eulerAngles(0, 1, 2);
}

void FwNmpc::gridMapCb(const grid_map_msgs::GridMap::ConstPtr& msg) // Grid map msg callback
{
	bool ret;

	// convert to eigen XXX: this could be super expensive.. copying the *entire global map every time
	ret = grid_map::GridMapRosConverter::fromMessage(msg, global_map_);
	if (ret) ROS_ERROR("grid map cb: failed to convert msg to eigen");

	// bound position inside map
	grid_map::Position pos_xy = x0_pos_.head(2).reverse();
	grid_map::boundPositionToRange(pos_xy, global_map_.getLength(), global_map_.getPosition());

	// get index of position
	grid_map::Index idx_pos;
	ret = global_map_.getIndex(pos_xy, idx_pos);
	if (ret) ROS_ERROR("grid map cb: position outside map"); // should not happen

	// get / bound local map origin
	int idx_local_origin_x = idx_pos(0) - (LEN_IDX_E-1)/2;
	int idx_local_origin_y = idx_pos(1) - (LEN_IDX_N-1)/2;
	grid_map::boundIndexToRange(idx_local_origin_x, global_map_.getSize().x()-LEN_IDX_E);
	grid_map::boundIndexToRange(idx_local_origin_y, global_map_.getSize().y()-LEN_IDX_N);

	idx_pos = idx_local_origin_x+LEN_IDX_E, idx_local_origin_y+LEN_IDX_N;
	global_map_.getPosition(idx_pos, pos_xy);
	od_(OD_TERR_ORIG_E) = pos_xy(0);
	od_(OD_TERR_ORIG_N) = pos_xy(1);

	terr_data_ = global_map_["elevation"].block(idx_local_origin_x,idx_local_origin_y,LEN_IDX_E,LEN_IDX_N).transpose();
}

void FwNmpc::homePosCb(const mavros_msgs::HomePosition::ConstPtr& msg) // Home position msg callback
{
	home_lat_ = (double)msg->latitude;
	home_lon_ = (double)msg->longitude;
	home_alt_ = (double)msg->altitude;
}

void FwNmpc::localPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg) // Local position msg callback
{
	tf::poseMsgToEigen(msg->pose, x0_pos_);
}

void FwNmpc::localVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg) // Local velocity msg callback
{
	tf::vectorMsgToEigen(msg->twist.linear, x0_vel_);
}

void FwNmpc::sysStatusCb(const mavros_msgs::State::ConstPtr& msg) // System status msg callback
{
	// this message comes at 1 Hz and resets the NMPC if not in OFFBOARD mode
  ROS_INFO_STREAM("Received Pixhawk Mode: " <<  msg->mode);
  offboard_mode_ = msg->mode == "OFFBOARD";
}

void FwNmpc::windEstCb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg) // Wind estimation msg callback
{
	tf::vectorMsgToEigen(msg->twist.twist.linear, x0_wind_);
}

/* sets */

/* gets */

/* functions */

void FwNmpc::checkSubs()
{
	/* pull current waypoint list */
	ros::ServiceClient wp_pull_client_ = nmpc_.serviceClient<mavros_msgs::WaypointPull>("/mavros/mission/pull");
	mavros_msgs::WaypointPull wp_pull_srv;
	if (wp_pull_client_.call(wp_pull_srv)) {
		ROS_ERROR("check subs: received %d waypoints", (int)wp_pull_srv.response.wp_received);
	}
	else {
		ROS_ERROR("check subs: failed to call wp pull service");
	}

	/* wait for home waypoint */
	while (home_lat_ < 0.1 && home_lon_ < 0.1 ) {
		ros::spinOnce();
		ROS_INFO ("check subs: waiting for home position");
		sleep(0.5);
	}
	ROS_ERROR("check subs: received home position");

	return;
}

void FwNmpc::initAcadoVars()
{
	x0_pos_.setZero();
	x0_vel_ << 10.0, 0.0, 0.0;
	x0_euler_.setZero();
	x0_wind_.setZero();

	x0_.setZero();
	x_.setZero();
	u_.setZero();
	y_.setZero();
	yN_.setZero();
	W_.setZero();
	WN_.setZero();
	od_.setZero();

	updateAcadoX0();
	updateAcadoOd();
	updateAcadoY();
	updateAcadoW();


	Eigen::Map<Eigen::Matrix<double, ACADO_NX, ACADO_N + 1>>(const_cast<double*>(acadoVariables.x)) = x_;

	Eigen::Map<Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime>>


	Eigen::Map<Eigen::Matrix<double, ACADO_NOD-IDX_OD_TERR_DATA, 1>>(const_cast<double*>(acadoVariables.od + OD_TERR_DATA,
		ACADO_NOD-IDX_OD_TERR_DATA)) =
}

void FwNmpc::initHorizon()
{
	// initialize solver
	int RET = acado_initializeSolver();

	// initialize all nodes with forward simulation and zeroed out controls
	Eigen::Map<Eigen::Matrix<double, ACADO_NU, ACADO_N>>(const_cast<double*>(acadoVariables.u)) = u_.Zeros();
	initializeNodesByForwardSimulation();

	ROS_ERROR("re-init horizon: propagate states with zeroed controls through horizon");

	// if (re_init) {
	// 	// we are re-initializing the horizon possibly in-air, propagate current states forward
	//
	//
	// 	ROS_ERROR("re-init horizon: propagate states with zeroed controls through horizon");
	// }
	// else {
	// 	// this is the first initialization, the values don't really matter here...
	//
	// 	Eigen::Matrix<double, 1, ACADO_N+1> t_n1;
	// 	for (int i = 0; i < N+1; ++i) t_n1(i) = i;
	// 	Eigen::Matrix<double, 1, ACADO_N+1> ones_n1 = Eigen::Matrix<double, 1, ACADO_N+1>::Ones();
	//
	// 	/* states */
	//
	// 	// linearly propagate position at current velocity
	// 	x_.block(IDX_X_POS,0,3,N+1) = x0_.block(IDX_X_POS,0,3,0) * ones_n1 + t_step_ * (x0_vel_ - x0_wind_) * t_n1;
	//
	// 	// hold direction constant (corresponding to linear propagation of position)
	// 	x_.block(IDX_X_GAMMA,0,0,N+1) = x0_(IDX_X_GAMMA) * ones_n1;
	// 	x_.block(IDX_X_XI,0,0,N+1) = x0_(IDX_X_XI) * ones_n1;
	//
	// 	// assume zero roll (no lateral acceleration)
	// 	x_.block(IDX_X_PHI,0,0,N+1) = Eigen::Matrix<double, 1, ACADO_N+1>::Zero();
	//
	// 	/* controls */
	// 	u_.block(IDX_X_GAMMA,0,0,N) = constrain(x0_(IDX_X_GAMMA), lb_ * ones_n1();
	// 	u_.block(IDX_X_PHI,0,0,N) = Eigen::Matrix<double, 1, ACADO_N+1>::Zero();
	//
	// 	ROS_ERROR("init horizon: hold states and controls constant through horizon");
	// }
}

int FwNmpc::initNMPC()
{
	nmpc_.getParam("/nmpc/loop_rate", loop_rate_); 	// iteration loop rate
	nmpc_.getParam("/nmpc/time_step", t_step_);			// get model discretization step

	/* initialize ACADO variables */
	initAcadoVars();
	ROS_ERROR("init nmpc: ACADO variables initialized");

	/* initialize the solver */
	initHorizon();

	return RET;
}

int FwNMPC::nmpcIteration()
{
	obctrl_status_ = 0;

	// elapsed time reusable var
	ros::Duration t_elapsed;

	/* <--START: nmpc iteration timer */
	ros::Time t_iter_start = ros::Time::now();

	// various timer initializations
	uint64_t t_ctrl = 0;		// time elapsed since last control action was published (stays zero if not in offboard mode)
	uint64_t t_solve = 0;		// time elapsed during nmpc preparation and feedback step (solve time)
	uint64_t t_update = 0; 	// time elapsed during array updates
	uint64_t t_iter = 0; 		// time elapsed from start to end of iteration step

	// initialize returns
	int RET[2] = {0, 0};

	/* <-- START : update timer */
	ros::Time t_update_start = ros::Time::now();

	updateAcadoX0();
	updateAcadoY();
	updateAcadoW();
	updateAcadoOd();

	if (re_init_horizon_) {
		initHorizon();
		re_init_horizon_ = false;
	}

	t_elapsed = ros::Time::now() - t_update_start;
	t_update = t_elapsed.toNSec()/1000;
	/* update time in us : END --> */

	/* <-- START : nmpc solve timer */
	ros::Time t_solve_start = ros::Time::now();

	// prepare first step
	RET[0] = preparationStep();
	if ( RET[0] != 0 ) {
		ROS_ERROR("nmpc iteration: preparation step error, code %d", RET[0]);
		obctrl_status_ = RET[0];
		re_init_horizon_ = true;
	}

	// perform the feedback step
	RET[1] = feedbackStep();
	if ( RET[1] != 0 ) {
		ROS_ERROR("nmpc iteration: feedback step error, code %d", RET[1]);
		obctrl_status_ = RET[1]; //TODO: find a way that doesnt overwrite the other one...
		re_init_horizon_ = true;
	}
	updateHorizons();

	t_elapsed = ros::Time::now() - t_solve_start;
	t_solve = t_elapsed.toNSec()/1000;
	/* solve time in us : END --> */

	/* publishing */

	// send controls to vehicle
	publishControls();
	ros::Duration t_elapsed = ros::Time::now() - t_lastctrl;
	t_ctrl = t_elapsed.toNSec()/1000;
	t_lastctrl = ros::Time::now(); // record directly after publishing control

	// store current acado variables
	publishAcadoVars();

	// record timing / nmpc info
	publishNmpcInfo(t_iter_start, t_ctrl, t_solve, t_update);

	return 0;
}

void FwNmpc::publishControls()
{
	//TODO: this should be interpolated in the event of t_step < t_iter

	// get control bounds
	double roll_lim_rad;
	nmpc_.getParam("/nmpc/veh/roll_lim_deg", roll_lim_rad); // sym.
	roll_lim_rad *= DEG_TO_RAD;
	double v;
	nmpc_.getParam("/nmpc/veh/v_sink", v); // pos down
	double gamma_lb = -asin(constrain(v / airsp_, 0.0, 1.0));
	nmpc_.getParam("/nmpc/veh/v_clmb", v); // pos up
	double gamma_ub = asin(constrain(v / airsp_, 0.0, 1.0));

	// apply first NU controls
	double roll_ref = acadoVariables.u[IDX_U_PHI]);
	double gamma_ref = acadoVariables.u[IDX_U_GAMMA];
	if (isnan(roll_ref) || isnan(gamma_ref)) {
		// probably solver issue
		roll_ref = 0.0;
		gamma_ref = 0.0;
		re_init_horizon_ = true;
		obctrl_status_ = 11; // 11 (arbitrarily) for NaNs -- TODO: enumerate
	}
	else {
		// constrain to bounds
		roll_ref = constrain(roll_ref, -roll_lim_rad, roll_lim_rad);
		gamma_ref = constrain(gamma_ref, gamma_lb, gamma_ub);
	}

	// thrust sp -- not actually used currently
	mavros_msgs::Thrust thrust_sp;
	thrust_sp.thrust = 0.0f;

	thrust_pub_.publish(thrust_sp);

	// attitude setpoint
	geometry_msgs::PoseStamped att_sp;
	att_sp.header.frame_id = "world";
	Eigen::AngleAxisd rollAngle(roll_ref, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(gamma_ref, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond att_sp_quat_eigen = yawAngle * pitchAngle * rollAngle;
	quaternionEigenToMsg(att_sp_quat_eigen, att_sp.pose.orientation);

	att_sp_pub_.publish(att_sp);
}

void FwNmpc::publishAcadoVars()
{
	fw_ctrl::AcadoVars acado_vars;

	/* measurements */
	Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>>(const_cast<float*>(acado_vars.x0)) = x0_;

	/* state horizons */
	Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>>(const_cast<float*>(acado_vars.n)) = acadoVariables.x;
	acado_vars.e = (float)acadoVariables.x;
	acado_vars.d = (float)acadoVariables.x;
	acado_vars.gamma = (float)acadoVariables.x;
	acado_vars.xi = (float)acadoVariables.x[NX * i + 5];
	acado_vars.phi = (float)acadoVariables.x[NX * i + 6];


	/* control horizons */
	for (int i=0; i<N; i++) {
		acado_vars.uT[i] = (float)acadoVariables.u[NU * i];
		acado_vars.phi_ref[i] = (float)acadoVariables.u[NU * i + 1];
		acado_vars.theta_ref[i] = (float)acadoVariables.u[NU * i + 2];
	}

	/* online data */
	acado_vars.pparam1 = (uint8_t)acadoVariables.od[0];
	acado_vars.pparam2 = (float)acadoVariables.od[1];
	acado_vars.pparam3 = (float)acadoVariables.od[2];
	acado_vars.pparam4 = (float)acadoVariables.od[3];
	acado_vars.pparam5 = (float)acadoVariables.od[4];
	acado_vars.pparam6 = (float)acadoVariables.od[5];
	acado_vars.pparam7 = (float)acadoVariables.od[6];
	acado_vars.pparam1_next = (uint8_t)acadoVariables.od[7];
	acado_vars.pparam2_next = (float)acadoVariables.od[8];
	acado_vars.pparam3_next = (float)acadoVariables.od[9];
	acado_vars.pparam4_next = (float)acadoVariables.od[10];
	acado_vars.pparam5_next = (float)acadoVariables.od[11];
	acado_vars.pparam6_next = (float)acadoVariables.od[12];
	acado_vars.pparam7_next = (float)acadoVariables.od[13];
	acado_vars.R_acpt = (float)acadoVariables.od[14];
	acado_vars.ceta_acpt = (float)acadoVariables.od[15];
	acado_vars.wn = (float)acadoVariables.od[16];
	acado_vars.we = (float)acadoVariables.od[17];
	acado_vars.wd = (float)acadoVariables.od[18];
	acado_vars.alpha_p_co = (float)acadoVariables.od[19];
	acado_vars.alpha_m_co = (float)acadoVariables.od[20];
	acado_vars.alpha_delta_co = (float)acadoVariables.od[21];
	acado_vars.T_b_lat = (float)acadoVariables.od[22];
	acado_vars.T_b_lon = (float)acadoVariables.od[23];
	acado_vars.ddot_clmb = (float)acadoVariables.od[24];
	acado_vars.ddot_sink = (float)acadoVariables.od[25];

	/* references */
	acado_vars.yref[0] = (float)acadoVariables.y[0];
	acado_vars.yref[1] = (float)acadoVariables.y[1];
	acado_vars.yref[2] = (float)acadoVariables.y[2];
	acado_vars.yref[3] = (float)acadoVariables.y[3];
	acado_vars.yref[4] = (float)acadoVariables.y[4];
	acado_vars.yref[5] = (float)acadoVariables.y[5];
	acado_vars.yref[6] = (float)acadoVariables.y[6];
	acado_vars.yref[7] = (float)acadoVariables.y[7];
	acado_vars.yref[8] = (float)acadoVariables.y[8];
	acado_vars.yref[9] = (float)acadoVariables.y[9];
	acado_vars.yref[10] = (float)acadoVariables.y[10];

	acado_vars_pub_.publish(acado_vars);
}

void FwNmpc::publishNmpcInfo(ros::Time t_iter_start, uint64_t t_ctrl, uint64_t t_solve, uint64_t t_update)
{
	fw_ctrl::NmpcInfo nmpc_info;

	// obctrl status
	nmpc_info.status = obctrl_status_;

	/* solver info */
	nmpc_info.kkt = (double)getKKT();
	nmpc_info.obj = (double)getObjective();

	// various elapsed times
	nmpc_info.t_ctrl = t_ctrl;		// time elapsed since last control action was published (stays zero if not in auto mode)
	nmpc_info.t_solve = t_solve;	// time elapsed during nmpc preparation and feedback step (solve time)
	nmpc_info.t_update = t_update;	// time elapsed during array updates

	// nmpc iteration time in us
	ros::Duration t_elapsed = ros::Time::now() - t_iter_start;
	nmpc_info.t_iter = (uint64_t)(t_elapsed.toNSec()/1000);

	nmpc_info_pub_.publish(nmpc_info);
}

void FwNmpc::updateAcadoOd()
{
	double od;

	// airspeed
	od_(IDX_OD_V) = airsp_;

	// wind
	od_.segment(IDX_OD_W,3) = x0_wind_;

	// path -- TODO: replace with actual pixhawk waypoint messages and management!
	nmpc_.getParam("/nmpc/path/b_n", od_(IDX_OD_B));
	nmpc_.getParam("/nmpc/path/b_e", od_(IDX_OD_B+1));
	nmpc_.getParam("/nmpc/path/b_d", od_(IDX_OD_B+2));
	nmpc_.getParam("/nmpc/path/gamma_p", od_(IDX_OD_GAMMA_P));
	nmpc_.getParam("/nmpc/path/chi_p", od_(IDX_OD_CHI_P);

	// guidance
	nmpc_.getParam("/nmpc/od/T_lat", od_(IDX_OD_T_LAT));
	nmpc_.getParam("/nmpc/od/T_lon", od_(IDX_OD_T_LON));

	// terrain
	nmpc_.getParam("/nmpc/od/delta_h", od_(IDX_OD_DELTA_H));
	// IDX_OD_TERR_ORIG_N,
	// IDX_OD_TERR_ORIG_E,
	// IDX_OD_TERR_DATA

	Eigen::Map<Eigen::Matrix<double, IDX_OD_TERR_DATA, 1>>(const_cast<double*>(acadoVariables.od, IDX_OD_TERR_DATA)) = od_;
}

void FwNmpc::updateAcadoW()
{
	// state weight scales
	nmpc_.getParam("/nmpc/w_scale/chi", W_scale_(IDX_Y_CHI));
	nmpc_.getParam("/nmpc/w_scale/gamma", W_scale_(IDX_Y_GAMMA));
	nmpc_.getParam("/nmpc/w_scale/h", W_scale_(IDX_Y_H));

	// control weight scales
	nmpc_.getParam("/nmpc/w_scale/gamma_ref", W_scale_(IDX_Y_GAMMA_REF));
	nmpc_.getParam("/nmpc/w_scale/phi_ref", W_scale_(IDX_Y_PHI_REF));
	nmpc_.getParam("/nmpc/w_scale/gamma_rate", W_scale_(IDX_Y_GAMMA_RATE));
	nmpc_.getParam("/nmpc/w_scale/phi_rate", W_scale_(IDX_Y_PHI_RATE));

	Eigen::Matrix<double, ACADO_NY, 1> w_num;

	// state weights
	nmpc_.getParam("/nmpc/w/chi", w_num(IDX_Y_CHI));
	nmpc_.getParam("/nmpc/w/gamma", w_num(IDX_Y_GAMMA));
	nmpc_.getParam("/nmpc/w/h", w_num(IDX_Y_H));

	// control weights
	nmpc_.getParam("/nmpc/w/gamma_ref", w_num(IDX_Y_GAMMA_REF));
	nmpc_.getParam("/nmpc/w/phi_ref", w_num(IDX_Y_PHI_REF));
	nmpc_.getParam("/nmpc/w/gamma_rate", w_num(IDX_Y_GAMMA_RATE));
	nmpc_.getParam("/nmpc/w/phi_rate", w_num(IDX_Y_PHI_RATE));

	W_ = w_num.cwiseQuotient(W_scale_.pow(2));

	Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_NY>>(const_cast<double*>(acadoVariables.W)) = W_.asDiagonal();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, ACADO_NYN>>(const_cast<double*>(acadoVariables.WN)) = WN_.head(ACADO_NYN).asDiagonal();
}

void FwNmpc::updateAcadoX0();
{
	/* position */
	x0_.segment(IDX_X_POS,3) = x0_pos_;

	/* velocity axis */
	Eigen::Vector3d airsp_vec = x0_vel_ - x0_wind_;
	airsp_ = airsp_vec.norm();
	if (airsp < 4.0) {
		// airspeed is too low - bound it to minimum value
		airsp_ = 4.0;
		if (airsp_vec.norm() < 0.01) {
			// define airspeed vector in yaw direction with zero flight path angle
			x0_(IDX_X_GAMMA) = 0.0;
			x0_(IDX_X_XI) = x0_euler_(2);
			x0_(IDX_X_PHI) = x0_euler_(0);
		}
		else {
			airsp_vec = airsp_*airsp_vec.normalized();
			x0_(IDX_X_GAMMA) = -asin(airsp_vec(2)/airsp_);
			x0_(IDX_X_XI) = atan2(airsp_vec(1),airsp_vec(0));
			x0_(IDX_X_PHI) = x0_euler_(0);
		}
	}
	else {
		x0_(IDX_X_GAMMA) = -asin(airsp_vec(2)/airsp_);
		x0_(IDX_X_XI) = atan2(airsp_vec(1),airsp_vec(0));
		x0_(IDX_X_PHI) = x0_euler_(0);
	}

	Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>>(const_cast<double*>(acadoVariables.x0)) = x0_;
}

void FwNmpc::updateAcadoY()
{
	y_.setZero();
	yN_.setZero();
	Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) = y_;
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) = yN_;
}

void FwNmpc::updateHorizons()
{
	Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) = y_;
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) = yN_;
}

double FwNmpc::constrain(const double x, const double xmin, const double xmax)
{
	return (x<xmax) ? ( (x>xmin) ? x : xmin ) : xmax;
}

void FwNmpc::shutdown() {
	ROS_INFO("Shutting down NMPC...");
	ros::shutdown();
}

int main(int argc, char **argv)
{

	/* initialize node */
	ros::init(argc, argv, "fw_nmpc");
	fw_nmpc::FwNmpc nmpc;

	ros::spinOnce();

	/* wait for required subscriptions */
	nmpc.checkSubs();

	/* initialize states, params, and solver */
	int ret = nmpc.initNmpc();
	if (ret != 0) {
		ROS_ERROR("init nmpc: error in qpOASES QP solver.");
		return 1;
	}

	/*
	 * nmpc loop
	 */
	ros::Rate nmpc_rate(loop_rate_);
	ROS_ERROR("fw nmpc: entering NMPC loop");
	while (ros::ok()) {

		/* empty callback queues */
		ros::spinOnce();

		/* nmpc iteration step */
		ret = nmpc.nmpcIteration();
		if (ret != 0) {
			ROS_ERROR("nmpc iteration: error in qpOASES QP solver.");
			return 1;
		}

		/* sleep */
		nmpc_rate.sleep();
	}

	ROS_ERROR("fw nmpc: closing...");
	return 0;
}
