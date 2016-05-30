#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/TwistStamped.h>

#include <fw_nmpc.h>
#include <math.h>
#include <string.h>

// INCLUDES for mavros
#include <mavros/WaypointPull.h>

using namespace fw_nmpc;

FwNMPC::FwNMPC() :
		LOOP_RATE(20.0), //MAGIC NUMBER
		t_lastctrl({0}),
		bModeChanged(false),
		last_ctrl_mode(0),
		prev_ctrl_horiz_({0}),
		prev_wp_idx_(-1),
		last_wp_idx_(-1)
{

	ROS_INFO("Instance of NMPC created");

	/* subscribers */
	aslctrl_data_sub_	= nmpc_.subscribe("/mavros/aslctrl/data", 1, &FwNMPC::aslctrlDataCb, this);
	glob_pos_sub_ 		= nmpc_.subscribe("/mavros/global", 1, &FwNMPC::globPosCb, this);
	ekf_ext_sub_			= nmpc_.subscribe("/mavros/aslekf_extended", 1, &FwNMPC::ekfExtCb, this);
	nmpc_params_sub_ 	= nmpc_.subscribe("/mavros/nmpc_params", 1, &FwNMPC::nmpcParamsCb, this);
	waypoint_list_sub_= nmpc_.subscribe("/mavros/waypoints", 1, &FwNMPC::waypointListCb, this);
	current_wp_sub_		= nmpc_.subscribe("/mavros/current_wp", 1, &FwNMPC::currentWpCb, this);
	home_wp_sub_			= nmpc_.subscribe("/mavros/home_wp", 1, &FwNMPC::homeWpCb, this);

	/* publishers */
	// to pixhawk
	att_sp_pub_				= nmpc_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel", 10, true);
	// for logging
	kkt_pub_					= nmpc_.advertise<std_msgs::Float32>("/nmpc/info/kkt",10,true);
	obj_pub_					= nmpc_.advertise<std_msgs::Float32>("/nmpc/info/obj",10,true);
	tsolve_pub_				= nmpc_.advertise<std_msgs::UInt64>("/nmpc/info/tsolve",10,true);
	tctrl_pub_				= nmpc_.advertise<std_msgs::UInt64>("/nmpc/info/tctrl",10,true);
	// augmented states
	intg_e_t_pub_			= nmpc_.advertise<std_msgs::Float32>("/nmpc/augm/intg_e_t",10,true);
	intg_e_chi_pub_		= nmpc_.advertise<std_msgs::Float32>("/nmpc/augm/intg_e_chi",10,true);
	sw_pub_						= nmpc_.advertise<std_msgs::Float32>("/nmpc/augm/sw",10,true);
}

void FwNMPC::aslctrlDataCb(const mavros::AslCtrlData::ConstPtr& msg) {

	subs_.aslctrl_data.header 				= msg->header;
	subs_.aslctrl_data.aslctrl_mode 	= msg->aslctrl_mode;
	subs_.aslctrl_data.YawAngle 			= msg->YawAngle;
	subs_.aslctrl_data.AirspeedRef		= msg->AirspeedRef;

}

void FwNMPC::globPosCb(const sensor_msgs::NavSatFix::ConstPtr& msg) {

	subs_.glob_pos.latitude = msg->latitude;
	subs_.glob_pos.longitude = msg->longitude;
}

void FwNMPC::ekfExtCb(const mavros::AslEkfExt::ConstPtr& msg) {

	subs_.ekf_ext.airspeed 			= msg->airspeed;
	subs_.ekf_ext.windSpeed 		= msg->windSpeed;
	subs_.ekf_ext.windDirection = msg->windDirection;
}

void FwNMPC::nmpcParamsCb(const mavros::AslNmpcParams::ConstPtr& msg) {

	subs_.nmpc_params.k_chi 		= msg->k_chi;
	subs_.nmpc_params.Qdiag 		= msg->Qdiag;
}

void FwNMPC::waypointListCb(const mavros::WaypointList::ConstPtr& msg) {

	subs_.waypoint_list.waypoints = msg->waypoints;
}

void FwNMPC::currentWpCb(const std_msgs::Int32::ConstPtr& msg) {

	subs_.current_wp.data = msg->data;
}

void FwNMPC::homeWpCb(const geographic_msgs::GeoPoint::ConstPtr& msg) {

	subs_.home_wp.latitude = msg->latitude;
	subs_.home_wp.longitude = msg->longitude;
	subs_.home_wp.altitude = msg->altitude;
}

int FwNMPC::initNMPC() {

	/* Initialize ACADO variables */
	initACADOVars();

	/* Initialize the solver. */
	int RET = initializeSolver();

	/* get loop rate */
	nmpc_.getParam("/nmpc/loop_rate", LOOP_RATE);

	return RET;
}

void FwNMPC::initACADOVars() {

	//TODO: maybe actually wait for all subscriptions to be filled here before initializing?

	/* put something reasonable here.. NOT all zeros, solver is initialized from here */
	double X[NX] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double U[NU] = {0.0};
	double OD[NOD] = {15.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double Y[NY] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double W[NY] = {0.0, 10.0, 0.0, 0.0, 10.0, 100.0};

	/* these set a constant value for each variable through the horizon */

	// states
	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NX; ++j) acadoVariables.x[ i * NX + j ] = X[ j ];
	}

	// controls
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < NU; ++j) acadoVariables.u[ i * NU + j ] = U[ j ];
	}

	// online data
	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NOD; ++j) {
			if (j >= NOD-NU) acadoVariables.od[ i * NOD + j ] = prev_ctrl_horiz_[ i * NU + j-(NOD-NU) ];
			else acadoVariables.od[ i * NOD + j ] = OD[ j ];
		}
	}

	// references
	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NY; ++j) acadoVariables.y[ i * NY + j ] = Y[ j ];
	}

	// weights
	for (int i = 0; i < N; ++i) {
		for (int ii = 0; ii < NY; ++ii) {
			for (int j = 0; j < NY; ++j) {
				if ( ii==j ) {
					acadoVariables.W[ (i + ii) * NY + j ] = ( ii<NY-1 ) ? W[ ii ] : W[ ii ] * (1.0 - (double)i / (double)N) * (1.0 - (double)i / (double)N); //TODO: could include optional power here
				}
				else acadoVariables.W[ (i + ii) * NY + j ] = 0;
			}
		}
	}

	for (int ii = 0; ii < NYN; ++ii) {
		for (int j = 0; j < NYN; ++j) {
			if ( ii==j ) {
				acadoVariables.WN[ (N + ii) * NYN + j ] = ( ii<NYN-1 ) ? W[ ii ] : 0.0;
			}
			else acadoVariables.WN[ (N + ii) * NYN + j ] = 0;
		}
	}
}

void FwNMPC::initHorizon() {

	// update home wp
	const double new_home_wp[2] = {subs_.home_wp.latitude, subs_.home_wp.longitude};
	paths_.setHomeWp( new_home_wp );
	prev_wp_idx_ = -1;

	//TODO: sliding window and/or low pass filter

	/* hold current states constant through horizion */ //TODO: could at least propagate position init with current velocity
	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NX-NX_AUGM; ++j) acadoVariables.x[ i * NX + j ] = acadoVariables.x0[ j ];
	}

	/* initialize augmented states */
	for (int i = 0; i < N + 1; ++i) {
		for (int j = NX_AUGM; j < NX; ++j) acadoVariables.x[ i * NX + j ] = 0.0; //TODO: NOT ALL NECESSARILY ZEROS
	}

	/* get current controls */
//	double U[NU] = {0.0};
	double U = 0.0; // MAGIC NUMBER.. do we actually want to start with the current bank angle?? .. or the command should probably stick to zero at start.

	/* saturate controls */
//	for (int i = 0; i < NU; ++i) {
//		if (U[i] < subs_.CTRL_SATURATION[i][0]) U[i] = subs_.CTRL_SATURATION[i][0];
//		if (U[i] > subs_.CTRL_SATURATION[i][1]) U[i] = subs_.CTRL_SATURATION[i][1];
//	}
	if (U < subs_.CTRL_SATURATION[0]) U = subs_.CTRL_SATURATION[0];
	if (U > subs_.CTRL_SATURATION[1]) U = subs_.CTRL_SATURATION[1];

	/* normalize controls */
//	for (int i = 0; i < NU; ++i)  U[ i ] = U[ i ] * subs_.CTRL_NORMALIZATION[ i ];
	U = U * subs_.CTRL_NORMALIZATION;

	/* hold current controls constant through horizon */
//	for (int i = 0; i < N; ++i) {
//		for (int j = 0; j < NU; ++j) acadoVariables.u[ i * NU + j ] = U[ j ];
//	}
	for (int i = 0; i < N; ++i) acadoVariables.u[ i ] = U;
}

void FwNMPC::updateACADO_X0() {

	double X0[NX];
	paths_.ll2NE(X0[0], X0[1], (double)subs_.glob_pos.latitude, (double)subs_.glob_pos.longitude); // n, e
	X0[2]		= (double)subs_.aslctrl_data.YawAngle;						// xi
	X0[3]		= acadoVariables.x0[3];														// intg_e_t
	X0[4]		= acadoVariables.x0[4];														// intg_e_chi
	X0[5]		= acadoVariables.x0[5];														// sw

	for (int i = 0; i < NX; ++i) acadoVariables.x0[ i ] = X0[ i ];
}

void FwNMPC::updateACADO_OD() {

	/* update online data */
	double OD[NOD];
//	OD[0] = (double)subs_.ekf_ext.airspeed;	// airspeed
	double tmp_airsp_ref = (double)subs_.aslctrl_data.AirspeedRef;
	if ( tmp_airsp_ref < 11.0 ) tmp_airsp_ref = 11.0; //TODO: remove hard-coding, set as params, or input from pixhawk velmin/max
	if ( tmp_airsp_ref > 20.0 ) tmp_airsp_ref = 20.0;
	OD[0] = tmp_airsp_ref;	// airspeed reference
	OD[1] = paths_.path_current.pparam1;
	OD[2] = paths_.path_current.pparam2;
	OD[3] = paths_.path_current.pparam3;
	OD[4] = paths_.path_current.pparam4;
	OD[5] = paths_.path_current.pparam5;
	OD[6] = paths_.path_current.pparam6;
	OD[7] = paths_.path_current.pparam7;
	OD[8] = paths_.path_current.pparam8;
	OD[9] = paths_.path_current.pparam9;
	OD[10] = paths_.path_next.pparam1;
	OD[11] = paths_.path_next.pparam2;
	OD[12] = paths_.path_next.pparam3;
	OD[13] = paths_.path_next.pparam4;
	OD[14] = paths_.path_next.pparam5;
	OD[15] = paths_.path_next.pparam6;
	OD[16] = paths_.path_next.pparam7;
	OD[17] = paths_.path_next.pparam8;
	OD[18] = paths_.path_next.pparam9;
	OD[19] = (double)subs_.ekf_ext.windSpeed * cos((double)subs_.ekf_ext.windDirection);
	OD[20] = (double)subs_.ekf_ext.windSpeed * sin((double)subs_.ekf_ext.windDirection);
	OD[21] = (double)subs_.nmpc_params.k_chi;
	// current_online_data[22] = mu_r_prev;

	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NOD; ++j) {
			if (j >= NOD-NU) acadoVariables.od[ i * NOD + j ] = prev_ctrl_horiz_[ i * NU + j-(NOD-NU) ];
			else acadoVariables.od[ i * NOD + j ] = OD[ j ];
		}
	}
}

void FwNMPC::updateACADO_Y() {

	/* update references */
	double Y[NY];
	Y[0] 	= 0.0;	// e_t
	Y[1] 	= 0.0;	// e_chi
	Y[2] 	= 0.0;	// intg_e_t
	Y[3] 	= 0.0;	// intg_e_chi
	Y[4] 	= 0.0; 	// mu_r
	Y[5] 	= 0.0;	// delta_mu_r_k

	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NY; ++j) acadoVariables.y[ i * NY + j ] = Y[ j ];
	}
}

void FwNMPC::updateACADO_W() {

	/* update objective gains */
	double W[NY];
	for (int i = 0; i < NY; i++) W[ i ] = (double)subs_.nmpc_params.Qdiag[ i ];

	for (int i = 0; i < N; ++i) {
		for (int ii = 0; ii < NY; ++ii) {
			for (int j = 0; j < NY; ++j) {
				if ( ii==j ) {
					acadoVariables.W[ (i + ii) * NY + j ] = ( ii<NY-1 ) ? W[ ii ] : W[ ii ] * (1.0 - (double)i / (double)N) * (1.0 - (double)i / (double)N); //TODO: could include optional power here
				}
				else acadoVariables.W[ (i + ii) * NY + j ] = 0;
			}
		}
	}

	for (int ii = 0; ii < NYN; ++ii) {
		for (int j = 0; j < NYN; ++j) {
			if ( ii==j ) {
				acadoVariables.WN[ (N + ii) * NYN + j ] = ( ii<NYN-1 ) ? W[ ii ] : 0.0;
			}
			else acadoVariables.WN[ (N + ii) * NYN + j ] = 0;
		}
	}
}

int FwNMPC::getLoopRate() {
	return LOOP_RATE;
}

void FwNMPC::reqSubs() { //TODO: extend this and/or change this to all subs/initializations

	/* initialize home wp and current wp seq structs while waiting for real data */
	subs_.home_wp.latitude = 0.0;
	subs_.home_wp.longitude = 0.0;
	subs_.current_wp.data = 0;

  /* pull current waypoint list */
	ros::ServiceClient wp_pull_client_ = nmpc_.serviceClient<mavros::WaypointPull>("/mission/WaypointPull");
	mavros::WaypointPull wp_pull_srv;

	  if (wp_pull_client_.call(wp_pull_srv))
	  {
	    ROS_INFO("fw_nmpc: received %iu waypoints", (uint32_t)wp_pull_srv.response.wp_received);
	  }
	  else
	  {
	    ROS_ERROR("fw_nmpc: failed to call wp pull service");
	  }

	/* wait for home waypoint */
	while ( subs_.home_wp.latitude < 0.1 && subs_.home_wp.longitude < 0.1 ) {

		ros::spinOnce();

		ROS_INFO ("fw_nmpc: waiting for home waypoint");

		sleep(0.5);
	}

	return;
}

int FwNMPC::nmpcIteration() {

	int RET[2] = {0, 0};

	/* check mode */ //TODO: should include some checking to make sure not on ground/ other singularity ridden cases
	bModeChanged = (subs_.aslctrl_data.aslctrl_mode - last_ctrl_mode) > 0 ? false : true;
	last_ctrl_mode = subs_.aslctrl_data.aslctrl_mode;
	if (subs_.aslctrl_data.aslctrl_mode == 5) {

		/* start timer */
		ros::Time t_start = ros::Time::now();

		if (bModeChanged) {
			/* first time in loop */
			initHorizon();
			bModeChanged = false;
		}

		/* update ACADO states/references/weights */
		updateACADO_X0();
		updateACADO_Y();
		updateACADO_W();

		/* check current waypoint */
		if ( subs_.current_wp.data != prev_wp_idx_ ) {
			// reset switch state, sw, horizon //TODO: encapsulate
			for (int i = 1; i < N + 2; ++i) {
				acadoVariables.x[ i * NX - 1 ] = 0.0;
			}
			acadoVariables.x0[ NX ] = 0.0;
		}

		/* update path */
		if ( subs_.current_wp.data != last_wp_idx_ ) {
			prev_wp_idx_ = last_wp_idx_;
		}
		paths_.updatePaths(subs_.waypoint_list, subs_.current_wp.data, prev_wp_idx_);
		last_wp_idx_ = subs_.current_wp.data;

		/* update ACADO online data */
		updateACADO_OD();

		/* Prepare first step */
		RET[0] = preparationStep();

		/* Perform the feedback step. */
		RET[1] = feedbackStep();

		/* store ctrl horizon before shift */
		memcpy(prev_ctrl_horiz_, acadoVariables.u, sizeof(acadoVariables.u)); //NOTE: only copies N sets of ctrls
		memcpy(prev_ctrl_horiz_+(N*NU), acadoVariables.u+((N-1)*NU), 8*NU); //NOTE: this assume doubles (8 bytes)

		/* publish control action */
		publishControls(subs_.aslctrl_data.header);

		//TODO: this should be interpolated in the event of Tnmpc ~= Tfcall //TODO: should this be before the feedback step?
		/* Optional: shift the initialization (look in acado_common.h). */
//		shiftStates(1, 0, 0);
		shiftControls( 0 );

		//TODO: this should be interpolated in the event of Tnmpc ~= Tfcall
		/* shift augmented states */
		for (int i = NX_AUGM; i < NX; ++i) acadoVariables.x0[i] = acadoVariables.x[i];

		/* publish augmented states */
		publishAugmStates();

		/* publish nmpc info */
		publishNmpcInfo(t_start);

	}

	/* return status */
	return (RET[0] != 0 || RET[1] != 0) ? 1 : 0; //perhaps a better reporting method?
}

void FwNMPC::publishControls(std_msgs::Header header) {

	//TODO: this should be interpolated in the event of Tnmpc ~= Tfcall
	/* Apply the NEXT control immediately to the process, first NU components. */
//	double ctrl[NU];
	double ctrl;
//	for (int i = 0; i < NU; ++i)  ctrl[ i ] = acadoVariables.u[ i ] / subs_.CTRL_NORMALIZATION[ i ];
	ctrl = acadoVariables.u[0] / subs_.CTRL_NORMALIZATION;

	/* saturate controls */
//	for (int i = 0; i < NU; ++i) {
//		if (ctrl[i] < subs_.CTRL_SATURATION[i][0]) ctrl[i] = subs_.CTRL_SATURATION[i][0];
//		if (ctrl[i] > subs_.CTRL_SATURATION[i][1]) ctrl[i] = subs_.CTRL_SATURATION[i][1];
//	}
	if (ctrl < subs_.CTRL_SATURATION[0]) ctrl = subs_.CTRL_SATURATION[0];
	if (ctrl > subs_.CTRL_SATURATION[1]) ctrl = subs_.CTRL_SATURATION[1];

	//NOTE: using cmd_vel members.. because dont want to convert to quaternions.
	geometry_msgs::TwistStamped att_sp_msg;
	att_sp_msg.twist.angular.x = ctrl; // mu
	att_sp_pub_.publish(att_sp_msg);

	/* update last control timestamp / publish elapsed ctrl loop time */
	ros::Duration t_elapsed = ros::Time::now() - t_lastctrl;
	uint64_t tctrl = t_elapsed.toNSec()/1000;
	tctrl_pub_.publish(tctrl);
	t_lastctrl = ros::Time::now();
}

void FwNMPC::publishAugmStates() {

	intg_e_t_pub_.publish( (float)acadoVariables.x0[NX-NX_AUGM] );
	intg_e_chi_pub_.publish( (float)acadoVariables.x0[NX-NX_AUGM+1] );
	sw_pub_.publish( (float)acadoVariables.x0[NX-NX_AUGM+2] );
}

void FwNMPC::publishNmpcInfo(ros::Time t_start) {

	/* solver info */
	kkt_pub_.publish( (float)getKKT() );
	obj_pub_.publish( (float)getObjective() );

	/* record elapsed time in us */
	ros::Duration t_elapsed = ros::Time::now() - t_start;
	uint64_t t_solve = t_elapsed.toNSec()/1000;
	tsolve_pub_.publish(t_solve);
}

void FwNMPC::shutdown() {
	ROS_INFO("Shutting down NMPC...");
	ros::shutdown();
}

int main(int argc, char **argv)
{

	/* initialize node */
	ros::init(argc, argv, "fw_nmpc");
	fw_nmpc::FwNMPC nmpc;

	ros::spinOnce();

	/* wait for required subscriptions */
	nmpc.reqSubs();

	/* initialize states, params, and solver */
	int ret = nmpc.initNMPC();

	if (ret != 0) {
		ROS_ERROR("initNMPC: error in qpOASES QP solver.");
		return 1;
	}

	/*
	 * NMPC loop
	 */
	double loop_rate = nmpc.getLoopRate();
	ros::Rate nmpc_rate(loop_rate);
	while (ros::ok()) {

		/* empty callback queues */
		ros::spinOnce();

		/* nmpc iteration step */
		ret = nmpc.nmpcIteration();

		if (ret != 0) {
			ROS_ERROR("nmpc_iteration: error in qpOASES QP solver.");
			return 1;
		}

		/* sleep */
		nmpc_rate.sleep();
	}

	ROS_INFO("fw_nmpc: closing...");

	return 0;
}
