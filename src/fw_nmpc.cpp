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
		loop_rate_(10.0),
		t_step_(0.1),
{
	ROS_INFO("Instance of NMPC created");

	/* subscribers */
	imu_sub_ = nmpc_.subscribe("/mavros/imu/data", 1, &FwNmpc::imuCb, this);
	grid_map_sub_ = nmpc_.subscribe("/fw_lander/global_map", 1, &FwNmpc::gridMapCb, this);
	grid_map_info_sub_ = nmpc_.subscribe("/fw_lander/global_map_info", 1, &FwNmpc::gridMapInfoCb, this);
	home_pos_sub_ = nmpc_.subscribe("/mavros/home_position", 1, &FwNmpc::homePosCb, this);
	local_pos_sub_ = nmpc_.subscribe("/mavros/local_position/pose", 1, &FwNmpc::localPosCb, this);
  local_vel_sub_ = nmpc_.subscribe("/mavros/local_position/velocity", 1, &FwNmpc::localVelCb, this);
	wind_est_sub_ = nmpc_.subscribe("/mavros/wind_estimation", 1, &FwNmpc::windEstCb, this);

	/* publishers */
	att_sp_pub_ = nmpc_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1);
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

	// convert to eigen
	ret = fromMessage(msg, global_map_);
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
	od_(OD_TERR_ORIG_E) = pos_xy(1);
	od_(OD_TERR_ORIG_N) = pos_xy(2);

	Eigen::Map<Eigen::Matrix<double, ACADO_NOD-IDX_OD_TERR_DATA, 1>>(const_cast<double*>(acadoVariables.od + OD_TERR_DATA,
		ACADO_NOD-IDX_OD_TERR_DATA)) = global_map_["elevation"].block(idx_local_origin_x,idx_local_origin_y,LEN_IDX_E,LEN_IDX_N).transpose();
}

void FwNmpc::gridMapInfoCb(const grid_map_msgs::GridMapInfo::ConstPtr& msg) // Grid map info msg callback
{

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
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) = y_;
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) = yN_;
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_NY>>(const_cast<double*>(acadoVariables.W)) = W_;
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, ACADO_NYN>>(const_cast<double*>(acadoVariables.WN)) = WN_;
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
	/* elapsed time reusable var */
	ros::Duration t_elapsed;

	/* start nmpc iteration timer --> */
	ros::Time t_iter_start = ros::Time::now();

	/* various timer initializations */
	uint64_t t_ctrl = 0;		// time elapsed since last control action was published (stays zero if not in auto mode)
	uint64_t t_solve = 0;		// time elapsed during nmpc preparation and feedback step (solve time)
	uint64_t t_update = 0; 	// time elapsed during array updates
	uint64_t t_wp_man = 0; 	// time elapsed during waypoint management

	/* initialize returns */
	int RET[2] = {0, 0};

	/* check mode */ //TODO: should include some checking to make sure not on ground/ other singularity ridden cases //TODO: this does not cover RC loss..
	if (last_ctrl_mode != 5) bModeChanged = true;
	last_ctrl_mode = subs_.aslctrl_data.aslctrl_mode;
	if (subs_.aslctrl_data.aslctrl_mode == 5) {

		/* start update timer --> */
		ros::Time t_update_start = ros::Time::now();

		int obctrl_status = 0;

		if (bModeChanged) {
			/* first time in loop */

			// re-initialize horizon / solver
			initHorizon();

			updateAcadoY();
			updateAcadoW();

			bModeChanged = false;
		}
		else {
			//regular update

			/* update ACADO states/references/weights */
			updateAcadoX0();
			updateAcadoY();
			updateAcadoW();
		}

		/* update time in us <-- */
		t_elapsed = ros::Time::now() - t_update_start;
		t_update = t_elapsed.toNSec()/1000;

		/* start waypoint management timer --> */
		ros::Time t_wp_man_start = ros::Time::now();

		/* check current waypoint */
		if ( subs_.current_wp.data != last_wp_idx_ ) {

			// reset switch state, sw, horizon //TODO: encapsulate
			for (int i = 1; i < N + 2; ++i) acadoVariables.x[ i * NX - 1 ] = 0.0;
			acadoVariables.x0[ 12 ] = 0.0;
		}
		last_wp_idx_ = subs_.current_wp.data;

		/* update path */
		paths_.updatePaths(subs_.waypoint_list, subs_.current_wp.data);

		/* update ACADO online data */
		updateAcadoOd();

		/* waypoint management time in us <-- */
		t_elapsed = ros::Time::now() - t_wp_man_start;
		t_wp_man = t_elapsed.toNSec()/1000;

		/* start nmpc solve timer --> */
		ros::Time t_solve_start = ros::Time::now();

		/* Prepare first step */
		RET[0] = preparationStep();
		if ( RET[0] != 0 ) {
			ROS_ERROR("nmpc iteration: preparation step error, code %d", RET[0]);
			obctrl_status = RET[0];
		}

		/* Perform the feedback step. */
		RET[1] = feedbackStep();
		if ( RET[1] != 0 ) {
			ROS_ERROR("nmpc iteration: feedback step error, code %d", RET[1]);
			obctrl_status = RET[1]; //TODO: find a way that doesnt overwrite the other one...
		}

		/* solve time in us <-- */
		t_elapsed = ros::Time::now() - t_solve_start;
		t_solve = t_elapsed.toNSec()/1000;

		/* nmpc iteration time in us (approximate for publishing to pixhawk) <-- */
		t_elapsed = ros::Time::now() - t_iter_start;
		uint64_t t_iter_approx = t_elapsed.toNSec()/1000;

		/* publish control action */
		publishControls(t_ctrl, t_iter_approx, obctrl_status);

		//TODO: this should be interpolated in the event of Tnmpc ~= Tfcall //TODO: should this be before the feedback step?
		/* Optional: shift the initialization (look in acado_common.h). */
//		shiftStates(1, 0, 0);
//		shiftControls( 0 );

	}
	else {
		// send "alive" status

		/* publish obctrl msg */
		mavros_msgs::AslObCtrl obctrl_msg;
		obctrl_msg.timestamp = 0;
		obctrl_msg.uThrot = 0.0f;
		obctrl_msg.uThrot2 = 0.0f; // for monitoring on QGC
		obctrl_msg.uAilR = 0.0f;
		obctrl_msg.uAilL = 0.0f; // for monitoring on QGC
		obctrl_msg.uElev = 0.0f;
		obctrl_msg.obctrl_status = 0;

		obctrl_pub_.publish(obctrl_msg);
	}

	/* publish ACADO variables */ // should this go outside?
	publishAcadoVars();

	/* publish nmpc info */
	publishNmpcInfo(t_iter_start, t_ctrl, t_solve, t_update, t_wp_man);

	/* return status */
	return (RET[0] != 0 || RET[1] != 0) ? 1 : 0; //perhaps a better reporting method?
}

void FwNmpc::publishControls(uint64_t &t_ctrl, uint64_t t_iter_approx, int obctrl_status)
{

}

void FwNmpc::publishAcadoVars()
{

}

void FwNmpc::publishNmpcInfo(ros::Time t_iter_start, uint64_t t_ctrl, uint64_t t_solve, uint64_t t_update, uint64_t t_wp_man)
{

}

void FwNmpc::updateAcadoOd()
{
	double od;

	// airspeed
	od_(IDX_OD_V) = airsp_;

	// wind
	od_.segment(IDX_OD_W,3) = x0_wind_;

	// path
	// IDX_OD_B
	// IDX_OD_GAMMA_P
	// IDX_OD_CHI_P

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

}

void FwNmpc::updateAcadoX0()
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
