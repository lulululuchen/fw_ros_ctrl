/*******************************************************************************
 * Copyright (c) 2019, Thomas Stastny, Autonomous Systems Lab (ASL), ETH Zurich,
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

using namespace fw_nmpc;

FwNmpc::FwNmpc() :
		flaps_normalized_(0.0),
		home_lat_(0),
		home_lon_(0),
		home_alt_(0),
		offboard_mode_(false),
		px4_throt_(0.0),
		static_pres_(101325.0),
		temp_c_(25.0),
		n_prop_virt_(0.0),
		first_yaw_received_(false),
		last_yaw_msg_(0.0),
		len_sliding_window_(10),
		loop_rate_(10.0),
		t_step_(0.1)
{
	ROS_INFO("Instance of NMPC created");

	/* subscribers */
	act_sub_ = nmpc_.subscribe("/mavros/target_actuator_control", 1, &FwNmpc::actCb, this);
	grid_map_sub_ = nmpc_.subscribe("/elevation_mapper/map_mpc", 1, &FwNmpc::gridMapCb, this);
	home_pos_sub_ = nmpc_.subscribe("/mavros/home_position", 1, &FwNmpc::homePosCb, this);
	imu_sub_ = nmpc_.subscribe("/mavros/imu/data", 1, &FwNmpc::imuCb, this);
	local_pos_sub_ = nmpc_.subscribe("/mavros/local_position/pose", 1, &FwNmpc::localPosCb, this);
  local_vel_sub_ = nmpc_.subscribe("/mavros/local_position/velocity", 1, &FwNmpc::localVelCb, this);
	static_pres_sub_ = nmpc_.subscribe("/mavros/static_pressure", 1, &FwNmpc::staticPresCb, this);
	sys_status_sub_ = nmpc_.subscribe("/mavros/state", 1, &FwNmpc::sysStatusCb, this);
	temp_c_sub_ = nmpc_.subscribe("/mavros/temperature_baro", 1, &FwNmpc::tempCCb, this);
	wind_est_sub_ = nmpc_.subscribe("/mavros/wind_estimation", 1, &FwNmpc::windEstCb, this);

	/* publishers */
	att_sp_pub_ = nmpc_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1);
	nmpc_info_pub_ = nmpc_.advertise<fw_ctrl::NmpcInfo>("/nmpc/info",10,true);
	nmpc_meas_pub_	= nmpc_.advertise<fw_ctrl::NmpcMeasurements>("/nmpc/measurements",10,true);
	nmpc_states_pub_	= nmpc_.advertise<fw_ctrl::NmpcStates>("/nmpc/states",10,true);
	nmpc_controls_pub_	= nmpc_.advertise<fw_ctrl::NmpcControls>("/nmpc/controls",10,true);
	nmpc_online_data_pub_	= nmpc_.advertise<fw_ctrl::NmpcOnlineData>("/nmpc/online_data",10,true);
	nmpc_obj_ref_pub_	= nmpc_.advertise<fw_ctrl::NmpcObjRef>("/nmpc/obj_ref",10,true);
	nmpc_objN_ref_pub_	= nmpc_.advertise<fw_ctrl::NmpcObjNRef>("/nmpc/objN_ref",10,true);
	obctrl_status_pub_ = nmpc_.advertise<std_msgs::Int32>("/nmpc/status", 1);
	thrust_pub_ = nmpc_.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/thrust", 1);
}

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/* CALLBACKS  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void FwNmpc::actCb(const mavros_msgs::ActuatorControl::ConstPtr& msg) // actuator control target msg callback
{
	if (msg->group_mix == msg->PX4_MIX_FLIGHT_CONTROL) {
		px4_throt_ = (double)msg->controls[3];
		flaps_normalized_ = (double)msg->controls[4];
	}
} // actCb

void FwNmpc::gridMapCb(const grid_map_msgs::GridMap::ConstPtr& msg) // grid map msg callback
{
	bool ret;

	// get incoming terrain data
	ret = grid_map::GridMapRosConverter::fromMessage(msg, terrain_map_);
	if (ret) ROS_ERROR("grid map cb: failed to convert msg to eigen");

	// grid map center (assuming constant level-northing orientation in world frame)
	map_center_north_ = terrain_map_.info.pose.position.x; // N (inertial frame) = x (grid map frame) [m]
	map_center_east_ = -terrain_map_.info.pose.position.y; // E (inertial frame) = -y (grid map frame) [m]

	// get local terrain map origin (top-right corner of array)
	terr_local_origin_n_ = map_center_north_ - terrain_map_.info.length_x / 2.0;
	terr_local_origin_e_ = map_center_east_ - terrain_map_.info.length_y / 2.0;

	// map dimensions
	map_resolution_ = terrain_map_.info.resolution;
	map_height_ = terrain_map_.info.length_x / map_resolution_; //XXX: CHECK THIS.. not sure if length is res*Ncells, or res*Ncells-1
	map_width_ = terrain_map_.info.length_y / map_resolution_;
} // gridMapCb

void FwNmpc::homePosCb(const mavros_msgs::HomePosition::ConstPtr& msg) // home position msg callback
{
	home_lat_ = (double)msg->latitude;
	home_lon_ = (double)msg->longitude;
	home_alt_ = (double)msg->altitude;
} // homePosCb

void FwNmpc::imuCb(const sensor_msgs::Imu::ConstPtr& msg) // imu msg callback
{
	Eigen::Quaterniond q;
	tf::quaternionMsgToEigen(msg->orientation, q);
	x0_euler_ = q.toRotationMatrix().eulerAngles(0, 1, 2);
} // imuCb

void FwNmpc::localPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg) // local position msg callback
{
	// ENU -> NED
	x0_pos_(0) = msg->pose.position.y;
	x0_pos_(1) = msg->pose.position.x;
	x0_pos_(2) = -msg->pose.position.z;
} // localPosCb

void FwNmpc::localVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg) // local velocity msg callback
{
	// ENU -> NED
	x0_vel_(0) = msg->twist.linear.y;
	x0_vel_(1) = msg->twist.linear.x;
	x0_vel_(2) = -msg->twist.linear.z;
} // localVelCb

void FwNmpc::staticPresCb(const sensor_msgs::FluidPressure::ConstPtr& msg) // static pressure msg callback
{
	static_pres_ = msg->fluid_pressure;
} // staticPresCb

void FwNmpc::sysStatusCb(const mavros_msgs::State::ConstPtr& msg) // system status msg callback
{
	// this message comes at 1 Hz and resets the NMPC if not in OFFBOARD mode
  ROS_INFO_STREAM("Received Pixhawk Mode: " <<  msg->mode);
  offboard_mode_ = msg->mode == "OFFBOARD";
} // sysStatusCb

void FwNmpc::tempCCb(const sensor_msgs::Temperature::ConstPtr& msg) // temperature msg callback
{
	temp_c_ = msg->temperature;
} // tempCCb

void FwNmpc::windEstCb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg) // wind estimation msg callback
{
	// ACTUALLY one that is in NED already
	tf::vectorMsgToEigen(msg->twist.twist.linear, x0_wind_);
} // windEstCb

void FwNmpc::checkSubs()
{
	/* wait for home waypoint */
	while (home_lat_ < 0.1 && home_lon_ < 0.1) {
		ros::spinOnce();
		ROS_INFO ("check subs: waiting for home position");
		sleep(1.0);
	}
	ROS_ERROR("check subs: received home position");

	// XXX: should have checks (and/or timeouts) for the rest of the required state estimates

	return;
} // checkSubs

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/* HELPER FUNCTIONS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

double FwNmpc::calc_air_density()
{
	return static_pres_ / (AIR_GAS_CONST * (temp_c_ - ABSOLUTE_NULL_CELSIUS));
} // calc_air_density

double FwNmpc::flaps_to_rad(const double flaps_normalized)
{
	// for now, assuming that a full normalized signal corresponds to full actuation .. need to check this with px4 convention
	double flaps_lim;
	nmpc_.getParam("/nmpc/veh/flaps_lim", flaps_lim); // in deg
	flaps_lim *= DEG_TO_RAD;

	return flaps_lim * constrain_double(flaps_normalized, -1.0, 1.0);
} // flaps_to_rad

double FwNmpc::map_prop_speed_to_throt(const double n_prop, const double airsp, const double aoa)
{
	// prop speed input (converted from throttle input considering inflow and zero thrusting conditions)

	double vmin, vmax, n_T0_vmin, n_T0_vmax, n_max, epsilon_T;
	nmpc_.getParam("/nmpc/prop/vpmin", vpmin); 					// minimum propeller inflow [m/s]
	nmpc_.getParam("/nmpc/prop/vpmax", vpmax); 					// maximum propeller inflow [m/s]
	nmpc_.getParam("/nmpc/prop/n_T0_vmin", n_T0_vmin); 	// zero-thrust prop speed at minimum airspeed [rps]
	nmpc_.getParam("/nmpc/prop/n_T0_vmax", n_T0_vmax); 	// zero-thrust prop speed at maximum airspeed [rps]
	nmpc_.getParam("/nmpc/prop/n_max", n_max); 					// maximum prop speed [rps]
	nmpc_.getParam("/nmpc/prop/epsilon_T", epsilon_T); 	// thrust incidence [deg]
	epsilon_T *= DEG_TO_RAD; 														// thrust incidence [rad]

	const double vp = airsp * cos(aoa - epsilon_T); // inflow at propeller
	const double sig_vp = constrain_double((vp - vpmin) / (vpmax - vpmin), 0.0, 1.0); // prop inflow linear interpolater

	return (n_prop - n_T0_vmin * (1.0 - sig_vp) - n_T0_vmax * sig_vp)/(n_max - n_T0_vmin);
} // map_prop_speed_to_throt

double FwNmpc::map_px4_to_throt(const double px4_throt, const double airsp, const double aoa)
{
	double n_0, n_max, n_delta;
	nmpc_.getParam("/nmpc/prop/n_0", n_0); 							// prop speed at zero inflow and zero input [rps]
	nmpc_.getParam("/nmpc/prop/n_max", n_max); 					// maximum prop speed [rps]
	nmpc_.getParam("/nmpc/prop/n_delta", n_delta);			// slope of first order prop speed to throttle input (PX4) mapping approximation

	return map_prop_speed_to_throt(px4_throt * n_delta + n_0, airsp, aoa);
} // map_px4_to_throt

double FwNmpc::map_throt_to_prop_speed(const double throt, const double airsp, const double aoa)
{
	// prop speed input (converted from throttle input considering inflow and zero thrusting conditions)

	double vmin, vmax, n_T0_vmin, n_T0_vmax, n_max, epsilon_T;
	nmpc_.getParam("/nmpc/prop/vpmin", vpmin); 					// minimum propeller inflow [m/s]
	nmpc_.getParam("/nmpc/prop/vpmax", vpmax); 					// maximum propeller inflow [m/s]
	nmpc_.getParam("/nmpc/prop/n_T0_vmin", n_T0_vmin); 	// zero-thrust prop speed at minimum airspeed [rps]
	nmpc_.getParam("/nmpc/prop/n_T0_vmax", n_T0_vmax); 	// zero-thrust prop speed at maximum airspeed [rps]
	nmpc_.getParam("/nmpc/prop/n_max", n_max); 					// maximum prop speed [rps]
	nmpc_.getParam("/nmpc/prop/epsilon_T", epsilon_T); 	// thrust incidence [deg]
	epsilon_T *= DEG_TO_RAD;														// thrust incidence [rad]

	const double vp = airsp * cos(aoa - epsilon_T); // inflow at propeller
	const double sig_vp = constrain_double((vp - vpmin) / (vpmax - vpmin), 0.0, 1.0); // prop inflow linear interpolater

	return (n_T0_vmin + throt * (n_max - n_T0_vmin)) * (1.0 - sig_vp) + (n_T0_vmax + throt * (n_max - n_T0_vmax)) * sig_vp;
} // map_throt_to_prop_speed

double FwNmpc::map_throt_to_px4(const double throt, const double airsp, const double aoa)
{
	double n_0, n_max, n_delta;
	nmpc_.getParam("/nmpc/prop/n_0", n_0); 							// prop speed at zero inflow and zero input [rps]
	nmpc_.getParam("/nmpc/prop/n_max", n_max); 					// maximum prop speed [rps]
	nmpc_.getParam("/nmpc/prop/n_delta", n_delta);			// slope of first order prop speed to throttle input (PX4) mapping approximation

	return (map_throt_to_prop_speed(throt, airsp, aoa) - n_0) / n_delta; // NOTE: this does not include dead-zone or true zero (prop may slowly spin at zero input)
} // map_throt_to_px4

void FwNmpc::propagate_virt_prop_speed(const double throt, const double airsp, const double aoa)
{
	double tau_n_prop;
	nmpc_.getParam("/nmpc/od/tau_n_prop", tau_n_prop);
	n_prop_virt_ = (map_throt_to_prop_speed(throt, airsp, aoa) - n_prop_virt_) / tau_n_prop / loop_rate_;
} // propagate_virt_prop_speed

double FwNmpc::unwrap_heading(double yaw)
{
	// XXX: once slip is included.. need to disambiguate heading (xi) vs yaw notation abuse here..

	if (!first_yaw_received_) {
		last_yaw_msg_ = yaw;
		first_yaw_received_ = true;
		return yaw;
	}

	float delta_yaw = yaw - last_yaw_msg_;
	if ( delta_yaw < -M_PI ) delta_yaw = delta_yaw + TWO_PI;
	if ( delta_yaw > M_PI ) delta_yaw = delta_yaw - TWO_PI;

	yaw = yaw + delta_yaw;

	last_yaw_msg_ = yaw;
	return yaw;
} // unwrap_heading

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/* PUBLISHERS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void FwNmpc::publishControls(double u_T, double phi_ref, double theta_ref)
{
	// thrust sp
	mavros_msgs::Thrust thrust_sp;
	thrust_sp.thrust = constrain_double(map_throt_to_px4(u_T, acadoVariables.x[IDX_X_V], acadoVariables.x[IDX_X_THETA] - acadoVariables.x[IDX_X_GAMMA]), 0.0, 1.0);

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
} // publishControls

void FwNmpc::publishNmpcInfo(ros::Time t_iter_start, uint64_t t_ctrl, uint64_t t_ext_obj, uint64_t t_solve, uint64_t t_update)
{
	fw_ctrl::NmpcInfo nmpc_info;

	// obctrl status
	nmpc_info.status = obctrl_status_;

	/* solver info */
	nmpc_info.kkt = (double)getKKT();
	nmpc_info.obj = (double)getObjective();

	// various elapsed times
	nmpc_info.t_ctrl = t_ctrl;				// time elapsed since last control action was published (stays zero if not in auto mode)
	nmpc_info.t_ext_obj = t_ext_obj;
	nmpc_info.t_solve = t_solve;			// time elapsed during nmpc preparation and feedback step (solve time)
	nmpc_info.t_update = t_update;		// time elapsed during array updates

	// nmpc iteration time in us
	ros::Duration t_elapsed = ros::Time::now() - t_iter_start;
	nmpc_info.t_iter = (uint64_t)(t_elapsed.toNSec()/1000);

	nmpc_info_pub_.publish(nmpc_info);
} // publishNmpcInfo

void FwNmpc::publishNmpcStates()
{
	fw_ctrl::NmpcMeasurements nmpc_measurements;
	fw_ctrl::NmpcStates nmpc_states;
	fw_ctrl::NmpcControls nmpc_controls;
	fw_ctrl::NmpcOnlineData nmpc_online_data;
	fw_ctrl::NmpcObjRef nmpc_obj_ref;
	fw_ctrl::NmpcObjNRef nmpc_objN_ref;

	/* measurements */
	nmpc_measurements.n  = (float)x0_(0);
	nmpc_measurements.e  = (float)x0_(1);
	nmpc_measurements.d  = (float)x0_(2);
	nmpc_measurements.v  = (float)x0_(3);
	nmpc_measurements.gamma  = (float)x0_(4);
	nmpc_measurements.xi  = (float)x0_(5);
	nmpc_measurements.phi  = (float)x0_(6);
	nmpc_measurements.theta  = (float)x0_(7);
	nmpc_measurements.n_prop  = (float)x0_(8);

	/* state/control horizons */
	for (int i=0; i<N; i++) {
		nmpc_states.n[i]  = (float)acadoVariables.x[NX * i + IDX_X_POS];
		nmpc_states.e[i]  = (float)acadoVariables.x[NX * i + IDX_X_POS+1];
		nmpc_states.d[i]  = (float)acadoVariables.x[NX * i + IDX_X_POS+2];
		nmpc_states.v[i]  = (float)acadoVariables.x[NX * i + IDX_X_V];
		nmpc_states.gamma[i]  = (float)acadoVariables.x[NX * i + IDX_X_GAMMA];
		nmpc_states.xi[i]  = (float)acadoVariables.x[NX * i + IDX_X_XI];
		nmpc_states.phi[i]  = (float)acadoVariables.x[NX * i + IDX_X_PHI];
		nmpc_states.theta[i]  = (float)acadoVariables.x[NX * i + IDX_X_THETA];
		nmpc_states.n_prop[i]  = (float)acadoVariables.x[NX * i + IDX_X_NPROP];

		nmpc_controls.u_T[i] = (float)acadoVariables.u[NU * i + IDX_U_U_T];
		nmpc_controls.phi_ref[i] = (float)acadoVariables.u[NU * i + IDX_U_PHI_REF];
		nmpc_controls.theta_ref[i] = (float)acadoVariables.u[NU * i + IDX_U_THETA_REF];
	}
	nmpc_states.n[N]  = (float)acadoVariables.x[NX * N + IDX_X_POS];
	nmpc_states.e[N]  = (float)acadoVariables.x[NX * N + IDX_X_POS+1];
	nmpc_states.d[N]  = (float)acadoVariables.x[NX * N + IDX_X_POS+2];
	nmpc_states.v[N]  = (float)acadoVariables.x[NX * N + IDX_X_V];
	nmpc_states.gamma[N]  = (float)acadoVariables.x[NX * N + IDX_X_GAMMA];
	nmpc_states.xi[N]  = (float)acadoVariables.x[NX * N + IDX_X_XI];
	nmpc_states.phi[N]  = (float)acadoVariables.x[NX * N + IDX_X_PHI];
	nmpc_states.theta[N]  = (float)acadoVariables.x[NX * N + IDX_X_THETA];
	nmpc_states.n_prop[N]  = (float)acadoVariables.x[NX * N + IDX_X_NPROP];

	/* online data */
	nmpc_online_data.rho = (float)acadoVariables.od[IDX_OD_RHO];
	nmpc_online_data.wn = (float)acadoVariables.od[IDX_OD_W];
	nmpc_online_data.we = (float)acadoVariables.od[IDX_OD_W+1];
	nmpc_online_data.wd = (float)acadoVariables.od[IDX_OD_W+2];
	nmpc_online_data.tau_phi = (float)acadoVariables.od[IDX_OD_TAU_PHI];
	nmpc_online_data.tau_theta = (float)acadoVariables.od[IDX_OD_TAU_THETA];
	nmpc_online_data.k_phi = (float)acadoVariables.od[IDX_OD_K_PHI];
	nmpc_online_data.k_theta = (float)acadoVariables.od[IDX_OD_K_THETA];
	nmpc_online_data.tau_n = (float)acadoVariables.od[IDX_OD_TAU_N];
	nmpc_online_data.delta_F = (float)acadoVariables.od[IDX_OD_DELTA_F];
	for (int i=0; i<N+1; i++) { // all prior online data are held constant through the horizon
		nmpc_online_data.soft_aoa[i] = (float)acadoVariables.od[NOD * i + IDX_OD_SOFT_AOA];
		nmpc_online_data.jac_soft_aoa_0[i][i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_AOA];
		nmpc_online_data.jac_soft_aoa_1[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_AOA+1];
		nmpc_online_data.soft_h[i] = (float)acadoVariables.od[NOD * i + IDX_OD_SOFT_H];
		nmpc_online_data.jac_soft_h_0[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_H];
		nmpc_online_data.jac_soft_h_1[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_H+1];
		nmpc_online_data.jac_soft_h_2[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_H+2];
		nmpc_online_data.jac_soft_h_3[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_H+3];
		nmpc_online_data.soft_r[i] = (float)acadoVariables.od[NOD * i + IDX_OD_SOFT_R];
		nmpc_online_data.jac_soft_r_0[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R];
		nmpc_online_data.jac_soft_r_1[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R+1];
		nmpc_online_data.jac_soft_r_2[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R+2];
		nmpc_online_data.jac_soft_r_3[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R+3];
		nmpc_online_data.jac_soft_r_4[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R+4];
		nmpc_online_data.jac_soft_r_5[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R+5];
	}

	/* objective references */
	for (int i=0; i<N; i++) {
		nmpc_obj_ref.vn[i] = (float)acadoVariables.y[NY * i + IDX_Y_VN];
		nmpc_obj_ref.ve[i] = (float)acadoVariables.y[NY * i + IDX_Y_VE];
		nmpc_obj_ref.vd[i] = (float)acadoVariables.y[NY * i + IDX_Y_VD];
		nmpc_obj_ref.v[i] = (float)acadoVariables.y[NY * i + IDX_Y_V];
		nmpc_obj_ref.phi[i] = (float)acadoVariables.y[NY * i + IDX_Y_PHI];
		nmpc_obj_ref.theta[i] = (float)acadoVariables.y[NY * i + IDX_Y_THETA];
		nmpc_obj_ref.soft_aoa[i] = (float)acadoVariables.y[NY * i + IDX_Y_SOFT_AOA];
		nmpc_obj_ref.soft_h[i] = (float)acadoVariables.y[NY * i + IDX_Y_SOFT_H];
		nmpc_obj_ref.soft_r[i] = (float)acadoVariables.y[NY * i + IDX_Y_SOFT_R];

		nmpc_obj_ref.u_T[i] = (float)acadoVariables.y[NY * i + IDX_Y_U_T];
		nmpc_obj_ref.phi_ref[i] = (float)acadoVariables.y[NY * i + IDX_Y_PHI_REF];
		nmpc_obj_ref.theta_ref[i] = (float)acadoVariables.y[NY * i + IDX_Y_THETA_REF];
	}
	nmpc_objN_ref.vn = (float)acadoVariables.yN[IDX_Y_VN];
	nmpc_objN_ref.ve = (float)acadoVariables.yN[IDX_Y_VE];
	nmpc_objN_ref.vd = (float)acadoVariables.yN[IDX_Y_VD];
	nmpc_objN_ref.v = (float)acadoVariables.yN[IDX_Y_V];
	nmpc_objN_ref.phi = (float)acadoVariables.yN[IDX_Y_PHI];
	nmpc_objN_ref.theta = (float)acadoVariables.yN[IDX_Y_THETA];
	nmpc_objN_ref.soft_aoa = (float)acadoVariables.yN[IDX_Y_SOFT_AOA];
	nmpc_objN_ref.soft_h = (float)acadoVariables.yN[IDX_Y_SOFT_H];
	nmpc_objN_ref.soft_r = (float)acadoVariables.yN[IDX_Y_SOFT_R];

	// publish
	nmpc_meas_pub_.publish(nmpc_measurements);
	nmpc_states_pub_.publish(nmpc_states);
	nmpc_controls_pub_.publish(nmpc_controls);
	nmpc_online_data_pub_.publish(nmpc_online_data);
	nmpc_obj_ref_pub_.publish(nmpc_obj_ref);
	nmpc_objN_ref_pub_.publish(nmpc_objN_ref);
} // publishNmpcStates

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/* OBJECTIVE FUNCTIONS  - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void FwNmpc::preEvaluateObjectives()
{
	// TODO: organize this whole thing better..

	// TODO: should be a better way than needing to create these and pass as pointer every time..
	grid_map::Matrix& terrain_data_matrix = terrain_map_["elevation"].colwise().reverse(); // TODO: sync the format of the acado functions with grid map.. or use grid map in the solver functions themselves
	double terrain_data[map_height_*map_width_];
	Eigen::Map<Matrix<double,map_height_,map_width_,RowMajor>>(terrain_data, map_height_, map_width_) = terrain_data_matrix;

	/* update params */
	updateObjectiveParameters();

	/* sliding window operations */
	shiftOcclusionSlidingWindow();
	castRays(terrain_data);

	/* external objectives */
	sumOcclusionDetections();
	evaluateExternalObjectives(terrain_data);
} // preEvaluateObjectives

void FwNmpc::updateObjectiveParameters()
{
	// TODO: is there a way to only run through this when we know something is updated? e.g. especially with the "log" and "division" operations in the soft constraint params..
	// XXX: THIS FUNCTION IS DISGUSTING -> CLEAN THIS UP!

	// FOR the weight-dependent cost shaping params: log_sqrt_w_over_sig1_XXX, one_over_sqrt_w_XXX
	// NOTE: depends on the current (un-modulated) weighting - evaluate PRIOR to updating current weighting priorities
	// NOTE: if soft constraints become dependent on weighting of other soft constraints, this will need some re-working

	// sliding occlusion window
	nmpc_.getParam("/nmpc/len_slw", len_sliding_window_);
	len_sliding_window_ = constrain_int(len_sliding_window_, 1, LEN_SLIDING_WINDOW_MAX);

	// soft angle of attack parameters
	nmpc_.getParam("/nmpc/aoa_params/delta_aoa", aoa_params_[0]);
	aoa_params_[0] *= DEG_TO_RAD; 																					// angle of attack buffer [rad]
	nmpc_.getParam("/nmpc/aoa_params/aoa_m", aoa_params_[1]);
	aoa_params_[1] *= DEG_TO_RAD; 																					// angle of attack lower bound [rad]
	nmpc_.getParam("/nmpc/aoa_params/aoa_p", aoa_params_[2]);
	aoa_params_[2] *= DEG_TO_RAD; 																					// angle of attack upper bound [rad]

	double sig_aoa_1, log_sqrt_w_over_sig1_aoa, one_over_sqrt_w_aoa;
	nmpc_.getParam("/nmpc/aoa_params/sig_aoa_1", sig_aoa_1);
	if (sig_aoa_1 < MIN_EXPONENTIAL_COST) sig_aoa_1 = MIN_EXPONENTIAL_COST; // angle of attack minimum soft exponential cost at unit height = 1

	if (acadoVariables.W[NY * IDX_Y_SOFT_AOA + IDX_Y_SOFT_AOA] <= sig_aoa_1) {
    // disable
    log_sqrt_w_over_sig1_aoa = 0.0;
    one_over_sqrt_w_aoa = -1.0;
	}
	else {
    log_sqrt_w_over_sig1_aoa = log(sqrt(acadoVariables.W[NY * IDX_Y_SOFT_AOA + IDX_Y_SOFT_AOA] / sig_aoa_1));
    one_over_sqrt_w_aoa = 1.0 / sqrt(acadoVariables.W[NY * IDX_Y_SOFT_AOA + IDX_Y_SOFT_AOA]);
	}

	// soft nadir terrain parameters
	nmpc_.getParam("/nmpc/terrain_params/h_offset", terrain_params_[0]); 	// nadir terrain offset [m]
	nmpc_.getParam("/nmpc/terrain_params/delta_h", terrain_params_[1]); 	// nadir terrain buffer [m]

	double sig_h_1, log_sqrt_w_over_sig1_h, one_over_sqrt_w_h;
	nmpc_.getParam("/nmpc/terrain_params/sig_h_1", sig_h_1);
	if (sig_h_1 < MIN_EXPONENTIAL_COST) sig_h_1 = MIN_EXPONENTIAL_COST; 	// nadir terrain minimum soft exponential cost at unit height = 1

	if (acadoVariables.W[NY * IDX_Y_SOFT_H + IDX_Y_SOFT_H] <= sig_h_1) {
		// disable
		log_sqrt_w_over_sig1_h = 0.0;
		one_over_sqrt_w_h = -1.0;
	}
	else {
		log_sqrt_w_over_sig1_h_ = log(sqrt(acadoVariables.W[NY * IDX_Y_SOFT_H + IDX_Y_SOFT_H] / sig_h_1));
		one_over_sqrt_w_h_ = 1.0 / sqrt(acadoVariables.W[NY * IDX_Y_SOFT_H + IDX_Y_SOFT_H]);
	}

	// soft radial terrain parameters
	nmpc_.getParam("/nmpc/terrain_params/r_offset", terrain_params_[4]); 	// radial terrain offset [m]
	nmpc_.getParam("/nmpc/terrain_params/delta_r0", terrain_params_[5]); 	// radial terrain buffer (at zero relative velocity) [m]

	double k_r_offset, k_delta_r;
	nmpc_.getParam("/nmpc/terrain_params/k_r_offset", k_r_offset); 				// gain on relative velocity term of radial offset
	nmpc_.getParam("/nmpc/terrain_params/k_delta_r", k_delta_r); 					// gain on relative velocity term of radial buffer

	double sig_r_1, log_sqrt_w_over_sig1_r, one_over_sqrt_w_r;
	nmpc_.getParam("/nmpc/terrain_params/sig_r_1", sig_r_1);
	if (sig_r_1 < MIN_EXPONENTIAL_COST) sig_r_1 = MIN_EXPONENTIAL_COST; 	// radial terrain minimum soft exponential cost at unit height = 1

	if (acadoVariables.W[NY * IDX_Y_SOFT_R + IDX_Y_SOFT_R] <= sig_r_1) {
    // disable
    log_sqrt_w_over_sig1_r = 0.0;
    one_over_sqrt_w_r = -1.0;
	}
	else {
    log_sqrt_w_over_sig1_r = log(sqrt(acadoVariables.W[NY * IDX_Y_SOFT_R + IDX_Y_SOFT_R] / sig_r_1));
    one_over_sqrt_w_r = 1.0 / sqrt(acadoVariables.W[NY * IDX_Y_SOFT_R + IDX_Y_SOFT_R]);
	}

	/* populate (remainder of) aoa param array (XXX: used for lsq obj functions.. may want to get rid of this struct or at least index it in the future) */
	aoa_params_[3] = log_sqrt_w_over_sig1_aoa;
	aoa_params_[4] = one_over_sqrt_w_aoa;

	/* populate (remainder of) terrain param array (XXX: used for lsq obj functions.. may want to get rid of this struct or at least index it in the future) */
	terrain_params_[2] = log_sqrt_w_over_sig1_h;
	terrain_params_[3] = one_over_sqrt_w_h;
	terrain_params_[6] = k_r_offset / tan(constrain_double(roll_lim_, 0.01, M_PI_2)) / ONE_G; // node-wise ground speed squared is multiplied in the internal ACADO lsq objective to make a multiple of the minimum turning radius
	terrain_params_[7] = k_delta_r / tan(constrain_double(roll_lim_, 0.01, M_PI_2)) / ONE_G; // node-wise ground speed squared is multiplied in the internal ACADO lsq objective to make a multiple of the minimum turning radius
	terrain_params_[8] = log_sqrt_w_over_sig1_r;
	terrain_params_[9] = one_over_sqrt_w_r;

	log_sqrt_w_over_sig1_r_ = log_sqrt_w_over_sig1_r; // XXX: this is currently the only one needed outside the lsq_objective.c file..

	/* guidance parameters */
	nmpc_.getParam("/nmpc/guidance/T_lat", T_lat_);
	nmpc_.getParam("/nmpc/guidance/T_lon", T_lon_);
	nmpc_.getParam("/nmpc/guidance/gamma_app_max", gamma_app_max_);
	gamma_app_max_ *= DEG_TO_RAD;

	/* path reference */[b_n, b_e, b_d, Gamma_p, chi_p];
	nmpc_.getParam("/nmpc/path/path_type", path_type_);
	nmpc_.getParam("/nmpc/path/b_n", path_reference_[0]);
	nmpc_.getParam("/nmpc/path/b_e", path_reference_[1]);
	nmpc_.getParam("/nmpc/path/b_d", path_reference_[2]);
	nmpc_.getParam("/nmpc/path/Gamma_p", path_reference_[3]);
	path_reference_[3] *= DEG_TO_RAD;
	nmpc_.getParam("/nmpc/path/chi_p", path_reference_[4]);
	path_reference_[4] *= DEG_TO_RAD;

} // updateObjectiveParameters

void FwNmpc::shiftOcclusionSlidingWindow()
{
	/* shift detection window */
	for (int i = 0; i < len_sliding_window_; ++i) {
		occ_detect_slw_[i] = occ_detect_slw_[i+1];
		occ_slw_[i * NOCC + IDX_OCC_POS] = occ_slw_[(i+1) * NOCC + IDX_OCC_POS];
		occ_slw_[i * NOCC + IDX_OCC_POS+1] = occ_slw_[(i+1) * NOCC + IDX_OCC_POS+1];
		occ_slw_[i * NOCC + IDX_OCC_POS+2] = occ_slw_[(i+1) * NOCC + IDX_OCC_POS+2];
		occ_slw_[i * NOCC + IDX_OCC_NORMAL] = occ_slw_[(i+1) * NOCC + IDX_OCC_NORMAL];
		occ_slw_[i * NOCC + IDX_OCC_NORMAL+1] = occ_slw_[(i+1) * NOCC + IDX_OCC_NORMAL+1];
		occ_slw_[i * NOCC + IDX_OCC_NORMAL+2] = occ_slw_[(i+1) * NOCC + IDX_OCC_NORMAL+2];
	}
} // shiftOcclusionSlidingWindow

void FwNmpc::castRays(const double *terrain_data)
{
	/* cast rays along ground speed vector at every node */
	for (i = 0; i < N+1; ++i)
	{
		/* calculate speed states */
		const double v = acadoVariables.x[i * NX + IDX_X_V];
		const double gamma = acadoVariables.x[i * NX + IDX_X_GAMMA];
		const double xi = acadoVariables.x[i * NX + IDX_X_XI];
		const double w_n = acadoVariables.od[i * NOD + IDX_OD_W];
		const double w_e = acadoVariables.od[i * NOD + IDX_OD_W+1];
		const double w_d = acadoVariables.od[i * NOD + IDX_OD_W+2];
		double speed_states[12];
		calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);

		/* forward ray */
		double p_occ_fwd[3];
		double n_occ_fwd[3];
		double r_occ_fwd; // XXX: not currently used
		int occ_detected_fwd = 0;
		get_occ_along_gsp_vec(p_occ_fwd, n_occ_fwd, &r_occ_fwd, &occ_detected_fwd,
						acadoVariables.x + (i * ACADO_NX), speed_states, terrain_params_,
						terr_local_origin_n_, terr_local_origin_e_, map_height_, map_width_,
						map_resolution_, terrain_data);

		/* log detections */
		occ_detect_slw_[len_sliding_window_-1+i] = occ_detected_fwd;
		occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_POS] = p_occ_fwd[0];
		occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_POS+1] = p_occ_fwd[1];
		occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_POS+2] = p_occ_fwd[2];
		occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_NORMAL] = n_occ_fwd[0];
		occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_NORMAL+1] = n_occ_fwd[1];
		occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_NORMAL+2] = n_occ_fwd[2];
	}
} // castRays

void FwNmpc::sumOcclusionDetections()
{
	/* sum detections on sliding horizon window for each node */
	for (i = 0; i < ACADO_N+1; ++i)
	{
		/* init */
		double jac_sig_r_fwd[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
		double jac_r_unit[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
		double r_unit_min;
		bool f_min;
		int occ_count = 0;

		/* calculate speed states */
		const double v = acadoVariables.x[i * NX + IDX_X_V];
		const double gamma = acadoVariables.x[i * NX + IDX_X_GAMMA];
		const double xi = acadoVariables.x[i * NX + IDX_X_XI];
		const double w_n = acadoVariables.od[i * NOD + IDX_OD_W];
		const double w_e = acadoVariables.od[i * NOD + IDX_OD_W+1];
		const double w_d = acadoVariables.od[i * NOD + IDX_OD_W+2];
		double speed_states[12];
		calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);

		/* sum detections in sliding window */
		for (j = i; j < (i + len_sliding_window_); ++j)
		{
			if (occ_detect_slw_[j]>0) {
				add_unit_radial_distance_and_gradient(jac_r_unit, &r_unit_min, &f_min, &occ_count,
					occ_slw_ + (j * NOCC + IDX_OCC_POS), occ_slw_ + (j * NOCC + IDX_OCC_NORMAL),
					acadoVariables.x + (i * NX), speed_states, terrain_params_);
			}
		}

		/* calculate objective/jacobian */ // XXX: should probably have this encapsulated somehow
		prio_r_fwd = 1.0;
		if (occ_count>0) {
			if (f_min) {
				// linear
				sig_r_fwd = 1.0 - log_sqrt_w_over_sig1_r_ * r_unit_min;
				jac_sig_r_fwd[0] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[0] / occ_count;
				jac_sig_r_fwd[1] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[1] / occ_count;
				jac_sig_r_fwd[2] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[2] / occ_count;
				jac_sig_r_fwd[3] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[3] / occ_count;
				jac_sig_r_fwd[4] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[4] / occ_count;
				jac_sig_r_fwd[5] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[5] / occ_count;
			}
			else {
				// exponential
				sig_r_fwd = exp(-r_unit_min * log_sqrt_w_over_sig1_r_);
				jac_sig_r_fwd[0] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[0] / occ_count;
				jac_sig_r_fwd[1] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[1] / occ_count;
				jac_sig_r_fwd[2] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[2] / occ_count;
				jac_sig_r_fwd[3] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[3] / occ_count;
				jac_sig_r_fwd[4] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[4] / occ_count;
				jac_sig_r_fwd[5] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[5] / occ_count;
			}
			jac_sig_r_fwd[3] = 0.0; /* discourage mpc from using airspeed to combat costs */
			prio_r_fwd = constrain_double(r_unit_min, 0.0, 1.0);
		}
		prio_r_(i, IDX_PRIO_R_FWD) = prio_r_fwd;

		/* log cost and jacobian */
		od_(IDX_OD_SOFT_R, i) = sig_r_fwd;
		od_(IDX_OD_JAC_SOFT_R, i) = jac_sig_r_fwd[0];
		od_(IDX_OD_JAC_SOFT_R+1, i) = jac_sig_r_fwd[1];
		od_(IDX_OD_JAC_SOFT_R+2, i) = jac_sig_r_fwd[2];
		od_(IDX_OD_JAC_SOFT_R+3, i) = jac_sig_r_fwd[3];
		od_(IDX_OD_JAC_SOFT_R+4, i) = jac_sig_r_fwd[4];
		od_(IDX_OD_JAC_SOFT_R+5, i) = jac_sig_r_fwd[5];
	}
} // sumOcclusionDetections

void FwNmpc::evaluateExternalObjectives(const double *terrain_data)
{
	/* evaluate external objectives */
	double v_ray[3];
	for (i = 0; i < ACADO_N +1; ++i)
	{
		/* soft aoa constraint */
		calculate_aoa_objective(&sig_aoa, jac_sig_aoa, &prio_aoa, acadoVariables.x + (i * ACADO_NX), aoa_params_);

		od_(IDX_OD_SOFT_AOA, i) = sig_aoa;
		od_(IDX_OD_JAC_SOFT_AOA, i) = jac_sig_aoa[0];
		od_(IDX_OD_JAC_SOFT_AOA+1, i) = jac_sig_aoa[1];

		prio_aoa_(i) = prio_aoa;

		/* soft height constraint */
		double h_terr;
		calculate_height_objective(&sig_h, jac_sig_h, &prio_h, &h_terr, acadoVariables.x + (i * ACADO_NX), terrain_params_,
			terr_local_origin_n_, terr_local_origin_e_, map_height_, map_width_, map_resolution_, terrain_data);

		od_(IDX_OD_SOFT_H, i) = sig_h;
		od_(IDX_OD_JAC_SOFT_H, i) = jac_sig_h[0];
		od_(IDX_OD_JAC_SOFT_H+1, i) = jac_sig_h[1];
		od_(IDX_OD_JAC_SOFT_H+2, i) = jac_sig_h[2];
		od_(IDX_OD_JAC_SOFT_H+3, i) = jac_sig_h[3];

		prio_h_(i) = prio_h;

		/* calculate speed states */
		const double v = acadoVariables.x[i * ACADO_NX + IDX_X_V];
		const double gamma = acadoVariables.x[i * ACADO_NX + IDX_X_GAMMA];
		const double xi = acadoVariables.x[i * ACADO_NX + IDX_X_XI];
		const double w_n = acadoVariables.od[i * NOD + IDX_OD_W];
		const double w_e = acadoVariables.od[i * NOD + IDX_OD_W+1];
		const double w_d = acadoVariables.od[i * NOD + IDX_OD_W+2];
		double speed_states[12];
		calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);

		/* soft radial constraint */

		/* left ray */
		v_ray[0] = -speed_states[9];
		v_ray[1] = speed_states[10];
		v_ray[2] = 0.0;
		calculate_radial_objective(&sig_r_left, jac_sig_r_left, &r_occ_left, p_occ_fwd, n_occ_fwd, &prio_r_left, &occ_detected_left,
			v_ray, acadoVariables.x + (i * ACADO_NX), speed_states, terrain_params_, terr_local_origin_n_, terr_local_origin_e_,
			map_height_, map_width_, map_resolution_, terrain_data);

		/* right ray */
		v_ray[0] = speed_states[9];
		v_ray[1] = -speed_states[10];
		v_ray[2] = 0.0;
		calculate_radial_objective(&sig_r_right, jac_sig_r_right, &r_occ_right, p_occ_fwd, n_occ_fwd, &prio_r_right, &occ_detected_right,
			v_ray, acadoVariables.x + (i * ACADO_NX), speed_states, terrain_params_, terr_local_origin_n_, terr_local_origin_e_,
			map_height_, map_width_, map_resolution_, terrain_data);

		jac_sig_r[0] = jac_sig_r_left[0] + jac_sig_r_right[0];
		jac_sig_r[1] = jac_sig_r_left[1] + jac_sig_r_right[1];
		jac_sig_r[2] = jac_sig_r_left[2] + jac_sig_r_right[2];
		jac_sig_r[3] = jac_sig_r_left[3] + jac_sig_r_right[3];
		jac_sig_r[4] = jac_sig_r_left[4] + jac_sig_r_right[4];
		jac_sig_r[5] = jac_sig_r_left[5] + jac_sig_r_right[5];

		const double sig_r = sig_r_left + sig_r_right;

		/* add to objective cost / jacobian */
		od_(IDX_OD_SOFT_R, i) += sig_r;
		od_(IDX_OD_JAC_SOFT_R, i) += jac_sig_r[0];
		od_(IDX_OD_JAC_SOFT_R+1, i) += jac_sig_r[1];
		od_(IDX_OD_JAC_SOFT_R+2, i) += jac_sig_r[2];
		od_(IDX_OD_JAC_SOFT_R+3, i) += jac_sig_r[3];
		od_(IDX_OD_JAC_SOFT_R+4, i) += jac_sig_r[4];
		od_(IDX_OD_JAC_SOFT_R+5, i) += jac_sig_r[5];

		/* prioritization */
		occ_detected_fwd = occ_slw[(len_sliding_window_-1+i) * NOCC + OCC_DETECT_FWD];
		if (!(one_over_sqrt_w_r_<0.0) && (occ_detected_fwd + occ_detected_left + occ_detected_right>0)) {

			/* take minimum unit radial distance */
			prio_r = (prio_r_(i, IDX_PRIO_R_FWD) < prio_r_left) ? prio_r_(i, IDX_PRIO_R_FWD) : prio_r_left;
			prio_r = (prio_r < prio_r_right) ? prio_r : prio_r_right;
		}
		else {
			prio_r = 1.0;
		}
		prio_r_(i, IDX_PRIO_R_MAX) = prio_r;
		prio_r_(i, IDX_PRIO_R_LEFT) = prio_r_left;
		prio_r_(i, IDX_PRIO_R_RIGHT) = prio_r_right;

		/* velocity reference */
		double v_ref[3];
		calculate_velocity_reference(v_ref, &path_error_lat_, &path_error_lon_, acadoVariables.x + (i * NX), path_reference_, guidance_params_,
			speed_states, jac_sig_r, prio_r, path_type_);

		if (i < N) {
			acadoVariables.y[i * NY + IDX_Y_VN] = v_ref[0];
			acadoVariables.y[i * NY + IDX_Y_VE] = v_ref[1];
			acadoVariables.y[i * NY + IDX_Y_VD] = v_ref[2];
		}
		else {
			acadoVariables.yN[IDX_Y_VN] = v_ref[0];
			acadoVariables.yN[IDX_Y_VE] = v_ref[1];
			acadoVariables.yN[IDX_Y_VD] = v_ref[2];
		}
	}
} // evaluateExternalObjectives

void FwNmpc::prioritizeObjectives()
{
	// prioritizes select objectives by adapting specific weights (external to the model jacobian)

	// objective weighting through horizon (row-major array)
	for (int i=0; i<N; i++) {
		for (int j=IDX_Y_VN; j<IDX_Y_VD; j++) {
			acadoVariables.W[NY*NY*i+NY*j+j] *= prio_h_(i) * prio_r_(i,IDX_PRIO_R_MAX); // de-prioritize path following objectives when near terrain
		}
	}

	// end term objective weighting through horizon (row-major array)
	for (int j=IDX_Y_VN; j<IDX_Y_VD; j++) {
		acadoVariables.WN[NYN*j+j] *= prio_h_(i) * prio_r_(i,IDX_PRIO_R_MAX); // de-prioritize path following objectives when near terrain
	}
} // prioritizeObjectives

void FwNmpc::filterControlReference()
{
	nmpc_.getParam("/nmpc/filt/tau_u", tau_u_); // control reference time constant

	// first order filter on reference horizon driven by currently applied control held through horizon
	u_ref_ = (u_ * Eigen::Matrix<double, 1, N>::Ones() - u_ref_) / tau_u_ / loop_rate_ + u_ref_;

} // filterControlReference

void FwNmpc::filterTerrainCostJacobian()
{
	nmpc_.getParam("/nmpc/filt/tau_terr", tau_terr_); // terrain jacobian time constant

	// first order filter on terrain based jacobian
	for (int i=0; i<N+1; i++) {
		for (int j=IDX_OD_OBJ; j<NOD; j++) {
			acadoVariables.od[NOD * i + j] = (od_(j, i) - acadoVariables.od[NOD * i + j]) / tau_terr_ / loop_rate_ + acadoVariables.od[NOD * i + j];
		}
	}

} // filterControlReference

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/* NMPC FUNCTIONS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void FwNmpc::applyControl()
{
	// apply first NU controls
	double u_T = acadoVariables.u[IDX_U_U_T];
	double phi_ref = acadoVariables.u[IDX_U_PHI_REF];
	double theta_ref = acadoVariables.u[IDX_U_THETA_REF];
	if (isnan(u_T) || isnan(phi_ref) || isnan(theta_ref)) {
		// probably solver issue
		u_T = 0.0;
		roll_ref = 0.0;
		theta_ref = 0.0;
		re_init_horizon_ = true;
		obctrl_status_ = 11; // 11 (arbitrarily) for NaNs -- TODO: enumerate
	}
	else {
		// constrain to bounds XXX: adapt these if necessary once we start playing with the constraints
		u_T = constrain_double(u_T, lb_(IDX_U_U_T), ub_(IDX_U_U_T));
		roll_ref = constrain_double(roll_ref, lb_(IDX_U_PHI_REF), ub_(IDX_U_PHI_REF));
		theta_ref = constrain_double(theta_ref, lb_(IDX_U_THETA_REF), ub_(IDX_U_THETA_REF));
	}

	u_(IDX_U_U_T) = u_T;
	u_(IDX_U_PHI_REF) = phi_ref;
	u_(IDX_U_THETA_REF) = theta_ref;
}

int FwNMPC::nmpcIteration()
{
	obctrl_status_ = 0;

	// initialize returns
	int RET[2] = {0, 0};

	// check offboard status
	if (!offboard_mode_) re_init_horizon_ = true;

	/* updates */

	updateAcadoX0(); // new measurements
	updateAcadoW(); // new weights (not including prioritization)
	updateAcadoConstraints(); // new constraints
	if (!re_init_horizon_)	filterControlReference(); // from previously applied control

	if (re_init_horizon_) {
		initHorizon();
		re_init_horizon_ = false;
	}

	/* objective pre-evaluation */

	// pre-evaluate select objectives (external to ACADO model)
	preEvaluateObjectives();

	// objective prioritization
	prioritizeObjectives();

	// update objective references
	updateAcadoY();

	// update online data (*DEPENDS on preEvaluateObjectives)
	filterTerrainCostJacobian(); // time delayed terrain cost jacobians
	updateAcadoOD();

	/* ACADO solver functions */

	// SOLVER: prepare ACADO workspace for next iteration step
	RET[0] = preparationStep();
	if ( RET[0] != 0 ) {
		ROS_ERROR("nmpc iteration: preparation step error, code %d", RET[0]);
		obctrl_status_ = RET[0];
		re_init_horizon_ = true;
		first_yaw_received_ = false;
	}

	// SOLVER: perform the feedback step
	RET[1] = feedbackStep();
	if ( RET[1] != 0 ) {
		ROS_ERROR("nmpc iteration: feedback step error, code %d", RET[1]);
		obctrl_status_ = RET[1]; //TODO: find a way that doesnt overwrite the other one...
		re_init_horizon_ = true;
	}

	/* publishing */

	// send controls to vehicle
	applyControl();
	publishControls(u_(IDX_U_U_T), u_(IDX_U_PHI_REF), u_(IDX_U_THETA_REF));
	ros::Duration t_elapsed = ros::Time::now() - t_lastctrl;
	t_ctrl = t_elapsed.toNSec()/1000;
	t_lastctrl = ros::Time::now(); // record directly after publishing control

	// store current nmpc states
	publishNmpcStates();

	// record timing / nmpc info
	publishNmpcInfo(t_iter_start, t_ctrl, t_ext_obj, t_solve, t_update);

	return 0;
} // nmpcIteration

void FwNmpc::updateAcadoConstraints()
{
	// update hard constraints

	// lower bounds
	nmpc_.getParam("/nmpc/lb/u_T", lb_(IDX_U_U_T));
	nmpc_.getParam("/nmpc/lb/phi_ref", lb_(IDX_U_PHI_REF));
	lb_(IDX_U_PHI_REF) *= DEG_TO_RAD;
	nmpc_.getParam("/nmpc/lb/theta_ref", lb_(IDX_U_THETA_REF));
	lb_(IDX_U_THETA_REF) *= DEG_TO_RAD;

	// upper bounds
	nmpc_.getParam("/nmpc/ub/u_T", ub_(IDX_U_U_T));
	nmpc_.getParam("/nmpc/ub/phi_ref", ub_(IDX_U_PHI_REF));
	ub_(IDX_U_PHI_REF) *= DEG_TO_RAD;
	nmpc_.getParam("/nmpc/ub/theta_ref", ub_(IDX_U_THETA_REF));
	ub_(IDX_U_THETA_REF) *= DEG_TO_RAD;

	// TODO: adapt constraints as necessary, e.g. for landing

	for (int i=0; i<N; i++) {
			acadoVariables.lbValues[NU * i + IDX_U_U_T] = lb_(IDX_U_U_T));
			acadoVariables.lbValues[NU * i + IDX_U_PHI_REF] = lb_(IDX_U_PHI_REF));
			acadoVariables.lbValues[NU * i + IDX_U_THETA_REF] = lb_(IDX_U_THETA_REF));

			acadoVariables.ubValues[NU * i + IDX_U_U_T] = ub_(IDX_U_U_T));
			acadoVariables.ubValues[NU * i + IDX_U_PHI_REF] = ub_(IDX_U_PHI_REF));
			acadoVariables.ubValues[NU * i + IDX_U_THETA_REF] = ub_(IDX_U_THETA_REF));
	}

} // updateAcadoConstraints

void FwNmpc::updateAcadoOD()
{
	// air density
	od_.block(IDX_OD_RHO, 0, 1, N+1) = Eigen::Matrix<double, 1, N+1>::Ones() * calc_air_density();

	// wind
	od_.block(IDX_OD_W, 0, 3, N+1) = x0_wind_ * Eigen::Matrix<double, 1, N+1>::Ones();

	// control-augmented model dynamics
	double temp_param;
	nmpc_.getParam("/nmpc/od/tau_phi", temp_param);
	od_.block(IDX_OD_TAU_PHI, 0, 1, N+1) = Eigen::Matrix<double, 1, N+1>::Ones() * temp_param;
	nmpc_.getParam("/nmpc/od/tau_theta", temp_param);
	od_.block(IDX_OD_TAU_THETA, 0, 1, N+1) = Eigen::Matrix<double, 1, N+1>::Ones() * temp_param;
	nmpc_.getParam("/nmpc/od/k_phi", temp_param);
	od_.block(IDX_OD_K_PHI, 0, 1, N+1) = Eigen::Matrix<double, 1, N+1>::Ones() * temp_param;
	nmpc_.getParam("/nmpc/od/k_theta", temp_param);
	od_.block(IDX_OD_K_THETA, 0, 1, N+1) = Eigen::Matrix<double, 1, N+1>::Ones() * temp_param;

	// prop delay
	nmpc_.getParam("/nmpc/od/tau_n", temp_param);
	od_.block(IDX_OD_TAU_N, 0, 1, N+1) = Eigen::Matrix<double, 1, N+1>::Ones() * temp_param;

	// symmetric flaps setting
	od_.block(IDX_OD_DELTA_F, 0, 1, N+1) = Eigen::Matrix<double, 1, N+1>::Ones() * flaps_to_rad(flaps_normalized_);

	// NOTE: pre-evaluated objectives/jacobians set in updateAcadoODObjectives -- this should be run BEFORE this update function

	Eigen::Map<Eigen::Matrix<double, NOD, N+1>>(const_cast<double*>(acadoVariables.od)) = od_;
} // updateAcadoOD

void FwNmpc::updateAcadoW()
{
	// NOTE: this does not yet include any weight priorities

	// state squared output scales (inverse of -- save a few divisions..)
	nmpc_.getParam("/nmpc/inv_y_scale_sq/vn", inv_y_scale_sq_(IDX_Y_VN));
	nmpc_.getParam("/nmpc/inv_y_scale_sq/vd", inv_y_scale_sq_(IDX_Y_VD));
	nmpc_.getParam("/nmpc/inv_y_scale_sq/ve", inv_y_scale_sq_(IDX_Y_VE));
	nmpc_.getParam("/nmpc/inv_y_scale_sq/v", inv_y_scale_sq_(IDX_Y_V));
	nmpc_.getParam("/nmpc/inv_y_scale_sq/phi", inv_y_scale_sq_(IDX_Y_PHI));
	nmpc_.getParam("/nmpc/inv_y_scale_sq/theta", inv_y_scale_sq_(IDX_Y_THETA));
	nmpc_.getParam("/nmpc/inv_y_scale_sq/soft_aoa", inv_y_scale_sq_(IDX_Y_SOFT_AOA));
	nmpc_.getParam("/nmpc/inv_y_scale_sq/soft_h", inv_y_scale_sq_(IDX_Y_SOFT_H));
	nmpc_.getParam("/nmpc/inv_y_scale_sq/soft_r", inv_y_scale_sq_(IDX_Y_SOFT_R));
	// control squared output scales
	nmpc_.getParam("/nmpc/inv_y_scale_sq/u_T", inv_y_scale_sq_(IDX_Y_U_T));
	nmpc_.getParam("/nmpc/inv_y_scale_sq/phi_ref", inv_y_scale_sq_(IDX_Y_PHI_REF));
	nmpc_.getParam("/nmpc/inv_y_scale_sq/theta_ref", inv_y_scale_sq_(IDX_Y_THETA_REF));

	// state weights
	nmpc_.getParam("/nmpc/w/vn", w_(IDX_Y_VN));
	nmpc_.getParam("/nmpc/w/ve", w_(IDX_Y_VE));
	nmpc_.getParam("/nmpc/w/vd", w_(IDX_Y_VD));
	nmpc_.getParam("/nmpc/w/v", w_(IDX_Y_V));
	nmpc_.getParam("/nmpc/w/phi", w_(IDX_Y_PHI));
	nmpc_.getParam("/nmpc/w/theta", w_(IDX_Y_THETA));
	nmpc_.getParam("/nmpc/w/soft_aoa", w_(IDX_Y_SOFT_AOA));
	nmpc_.getParam("/nmpc/w/soft_h", w_(IDX_Y_SOFT_H));
	nmpc_.getParam("/nmpc/w/soft_r", w_(IDX_Y_SOFT_R));
	// control weights
	nmpc_.getParam("/nmpc/w/u_T", w_(IDX_Y_U_T));
	nmpc_.getParam("/nmpc/w/phi_ref", w_(IDX_Y_PHI_REF));
	nmpc_.getParam("/nmpc/w/theta_ref", w_(IDX_Y_THETA_REF));

	// objective weighting through horizon (row-major array)
	for (int i=0; i<N; i++) {
		for (int j=0; j<NY; j++) {
			acadoVariables.W[NY*NY*i+NY*j+j] = w_(j) * inv_y_scale_sq_(j);
		}
	}

	// end term objective weighting through horizon (row-major array)
	for (int j=0; j<NY; j++) {
		acadoVariables.WN[NYN*j+j] = w_(j) * inv_y_scale_sq_(j);
	}

} // updateAcadoW

void FwNmpc::updateAcadoX0()
{
	/* ASSUMPTIONS:
	 * - yaw is assumed = to heading (i.e. no sideslip)
	 * - we need to unwrap heading because it is integrated through the horizon
	 */

	// position
	x0_.segment(IDX_X_POS,3) = x0_pos_;

	// velocity axis
	Eigen::Vector3d airsp_vec = x0_vel_ - x0_wind_;
	const double airsp = airsp_vec.norm();
	double airsp_thres;
	nmpc_.getParam("/nmpc/veh/airsp_thres", airsp_thres)
	if (airsp < airsp_thres) {
		// airspeed is too low - bound it to minimum value
		x0_(IDX_X_V) = airsp_thres;
		if (airsp < 0.1) { // XXX: magic number
			// if reeeally zero - define airspeed vector in yaw direction with zero flight path angle
			x0_(IDX_X_GAMMA) = 0.0; // flight path angle
			x0_(IDX_X_XI) = unwrap_heading(x0_euler_(2)); // heading
		}
		else {
			x0_(IDX_X_GAMMA) = -asin(airsp_vec(2)/airsp); // flight path angle
			x0_(IDX_X_XI) = unwrap_heading(atan2(airsp_vec(1),airsp_vec(0))); // heading
		}
	}
	else {
		x0_(IDX_X_V) = airsp; // airspeed
		x0_(IDX_X_GAMMA) = -asin(airsp_vec(2)/airsp); // flight path angle
		x0_(IDX_X_XI) = unwrap_heading(atan2(airsp_vec(1),airsp_vec(0))); // heading
	}

	// attitude
	x0_(IDX_X_PHI) = x0_euler_(0); // roll
	x0_(IDX_X_THETA) = x0_euler_(1); // pitch

	// prop speed
	if (re_init_horizon_) {
		// take current state
		const double throt = map_px4_to_throt(px4_throt_, airsp, x0_(IDX_X_THETA) - x0_(IDX_X_GAMMA));
		n_prop_virt_ = map_throt_to_prop_speed(throt, airsp, x0_(IDX_X_THETA) - x0_(IDX_X_GAMMA));
	}
	else {
		// propagate from last state
		propagate_virt_prop_speed(u_(IDX_U_U_T), acadoVariables.x[IDX_X_V], acadoVariables.x[IDX_X_THETA] - acadoVariables.x[IDX_X_GAMMA]);
	}
	x0_(IDX_X_NPROP) = n_prop_virt_;

	Eigen::Map<Eigen::Matrix<double, NX, 1>>(const_cast<double*>(acadoVariables.x0)) = x0_;
} // updateAcadoX0

void FwNmpc::updateAcadoY()
{
	// NOTE: run AFTER pre-evaluation of objectives

	// airspeed reference
	nmpc_.getParam("/nmpc/y_ref/v",airsp_ref);
	y_.block(IDX_Y_V, 0, 1, N) = Eigen::Matrix<double, 1, N>::Ones() * airsp_ref;
	yN_(IDX_Y_V) = airsp_ref;

	// attitude reference
	y_.block(IDX_Y_PHI, 0, 1, N) = u_ref_.block(IDX_U_PHI_REF, 0, 1, N);
	y_.block(IDX_Y_THETA, 0, 1, N) = u_ref_.block(IDX_U_THETA_REF, 0, 1, N);
	yN_(IDX_Y_PHI) = u_ref_(IDX_U_PHI_REF, N);
	yN_(IDX_Y_THETA) = u_ref_(IDX_U_THETA_REF, N);

	// control objective references
	y_.block(IDX_Y_U_T, 0, 1, N) = u_ref_.block(IDX_U_U_T, 0, 1, N);
	y_.block(IDX_Y_PHI_REF, 0, 1, N) = u_ref_.block(IDX_U_PHI_REF, 0, 1, N);
	y_.block(IDX_Y_THETA_REF, 0, 1, N) = u_ref_.block(IDX_U_THETA_REF, 0, 1, N);

	// NOTE: unit ground velocity references set in objective pre-evaluation

	Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) = y_;
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) = yN_;
} // updateAcadoY

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/* INITIALIZATION - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void FwNmpc::initAcadoVars()
{
	x0_pos_.setZero();
	x0_vel_ << 15.0, 0.0, 0.0;
	x0_euler_.setZero();
	x0_wind_.setZero();

	x0_.setZero();
	u_.setZero();
	u_ref_.setZero();
	y_.setZero();
	yN_.setZero();
	inv_y_scale_sq_.setZero();
	w_.setZero();
	od_.setZero();

	occ_detect_slw_.setZero();
	occ_slw_.setZero();

	updateAcadoX0();
	updateAcadoConstraints();
	updateAcadoW();
	updateAcadoOD();
	updateAcadoY();
} // initAcadoVars

void FwNmpc::initHorizon()
{
	// initialize solver
	int RET = acado_initializeSolver();

	if (re_init_horizon_) {
		// we are re-initializing the horizon possibly in-air, propagate current state measurements forward

		for (int i = 0; i < N; ++i) {

			// propagate position linearly at current ground velocity
			for (int j = IDX_X_POS; j < IDX_X_POS+3; ++j) {
				acadoVariables.x[NX * i + j] = x0_pos_(j) + x0_vel_(j) * t_step_ * i;
			}

			// hold velocity axis constant through horizon
			acadoVariables.x[NX * i + IDX_X_V] = x0_(IDX_X_V);
			acadoVariables.x[NX * i + IDX_X_GAMMA] = x0_(IDX_X_GAMMA);
			acadoVariables.x[NX * i + IDX_X_XI] = x0_(IDX_X_XI);

			// hold attitude constant (within bounds) through horizon
			double roll = constrain_double(x0_(IDX_X_PHI), lb_(IDX_U_PHI_REF), ub_(IDX_U_PHI_REF));
			double pitch = constrain_double(x0_(IDX_X_THETA), lb_(IDX_U_THETA_REF), ub_(IDX_U_THETA_REF));
			acadoVariables.x[NX * i + IDX_X_PHI] = roll;
			acadoVariables.x[NX * i + IDX_X_THETA] = pitch;

			// hold current throttle constant through horizon
			double throt = map_px4_to_throt(px4_throt_, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
			double n_prop = map_throt_to_prop_speed(throt, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
			acadoVariables.x[NX * i + IDX_X_NPROP] = n_prop;

			// controls
			acadoVariables.u[NU * i + IDX_U_U_T] = throt;
			acadoVariables.u[NU * i + IDX_U_PHI_REF] = 0.0;
			acadoVariables.u[NU * i + IDX_U_THETA_REF] = 0.0;
		}

		// propagate position linearly at current ground velocity
		for (int j = IDX_X_POS; j < IDX_X_POS+3; ++j) {
			acadoVariables.x[NX * N + j] = x0_pos_(j) + x0_vel_(j) * t_step_ * N;
		}

		// hold velocity axis constant through horizon
		acadoVariables.x[NX * N + IDX_X_V] = x0_(IDX_X_V);
		acadoVariables.x[NX * N + IDX_X_GAMMA] = x0_(IDX_X_GAMMA);
		acadoVariables.x[NX * N + IDX_X_XI] = x0_(IDX_X_XI);

		// hold attitude constant (within bounds) through horizon
		double roll = constrain_double(x0_(IDX_X_PHI), lb_(IDX_U_PHI_REF), ub_(IDX_U_PHI_REF));
		double pitch = constrain_double(x0_(IDX_X_THETA), lb_(IDX_U_THETA_REF), ub_(IDX_U_THETA_REF));
		acadoVariables.x[NX * N + IDX_X_PHI] = roll;
		acadoVariables.x[NX * N + IDX_X_THETA] = pitch;

		// hold current throttle constant through horizon
		double throt = map_px4_to_throt(px4_throt_, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
		double n_prop = map_throt_to_prop_speed(throt, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
		acadoVariables.x[NX * N + IDX_X_NPROP] = n_prop;

		ROS_ERROR("re-init horizon: propagate states with zero attitude reference and constant throttle through horizon");
	}
	else {
		// this is the first initialization, the values don't really matter here...

		for (int i = 0; i < N; ++i) {

			// propagate position linearly at current ground velocity
			for (int j = IDX_X_POS; j < IDX_X_POS+3; ++j) {
				acadoVariables.x[NX * i + j] = x0_pos_(j) + x0_vel_(j) * t_step_ * i;
			}

			// hold velocity axis constant through horizon
			acadoVariables.x[NX * i + IDX_X_V] = x0_(IDX_X_V);
			acadoVariables.x[NX * i + IDX_X_GAMMA] = x0_(IDX_X_GAMMA);
			acadoVariables.x[NX * i + IDX_X_XI] = x0_(IDX_X_XI);

			// zero roll, assume zero angle of attack, and hold pitch constant
			acadoVariables.x[NX * i + IDX_X_PHI] = 0.0;
			acadoVariables.x[NX * i + IDX_X_THETA] = x0_(IDX_X_GAMMA);

			// prop off
			acadoVariables.x[NX * i + IDX_X_NPROP] = 0.0;

			// controls
			acadoVariables.u[NU * i + IDX_U_U_T] = 0.0;
			acadoVariables.u[NU * i + IDX_U_PHI_REF] = 0.0;
			acadoVariables.u[NU * i + IDX_U_THETA_REF] = 0.0;
		}

		// propagate position linearly at current ground velocity
		for (int j = IDX_X_POS; j < IDX_X_POS+3; ++j) {
			acadoVariables.x[NX * N + j] = x0_pos_(j) + x0_vel_(j) * t_step_ * N;
		}

		// hold velocity axis constant through horizon
		acadoVariables.x[NX * N + IDX_X_V] = x0_(IDX_X_V);
		acadoVariables.x[NX * N + IDX_X_GAMMA] = x0_(IDX_X_GAMMA);
		acadoVariables.x[NX * N + IDX_X_XI] = x0_(IDX_X_XI);

		// zero roll, assume zero angle of attack, and hold pitch constant
		acadoVariables.x[NX * N + IDX_X_PHI] = 0.0;
		acadoVariables.x[NX * N + IDX_X_THETA] = x0_(IDX_X_GAMMA);

		// prop off
		acadoVariables.x[NX * N + IDX_X_NPROP] = 0.0;

		ROS_ERROR("init horizon: hold states and controls constant through horizon");
	}

} // initHorizon

int FwNmpc::initNMPC()
{
	nmpc_.getParam("/nmpc/loop_rate", loop_rate_); 	// iteration loop rate [Hz]
	nmpc_.getParam("/nmpc/time_step", t_step_);			// get model discretization step [s]

	/* initialize ACADO variables */
	initAcadoVars();
	ROS_ERROR("init nmpc: NMPC states initialized");

	/* initialize the solver */
	initHorizon();

	return RET;
}

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/* ROS NODE FUNCTIONS - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void FwNmpc::shutdown() {
	ROS_INFO("Shutting down NMPC...");
	ros::shutdown();
} // shutdown

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
} // main
