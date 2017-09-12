#include <ros/ros.h>
#include <ros/package.h>

#include <fw_nmpc.h>

#include <math.h>
#include <string.h>

#include <std_msgs/UInt8.h>

// INCLUDES for mavros
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/AslObCtrl.h>

// INCLUDES for fw_ctrl
#include <fw_ctrl/AcadoVars.h>
#include <fw_ctrl/NmpcInfo.h>

using namespace fw_nmpc;

FwNMPC::FwNMPC() :
		LOOP_RATE(20.0), //MAGIC NUMBER
		TSTEP(0.1), //MAGIC NUMBER
		FAKE_SIGNALS(0),
		t_lastctrl{0},
		bModeChanged(false),
		last_ctrl_mode(0),
		obctrl_en_(0),
		prev_wp_idx_(-1),
		last_wp_idx_(-1),
		bYawReceived(false),
		last_yaw_msg_(0.0f),
		track_error_lat_(0.0f),
		track_error_lon_(0.0f),
		W_scale_{0}
{

	ROS_INFO("Instance of NMPC created");

	/* subscribers */
	aslctrl_data_sub_	= nmpc_.subscribe("/mavros/aslctrl/data", 1, &FwNMPC::aslctrlDataCb, this);
	glob_pos_sub_ 		= nmpc_.subscribe("/mavros/global_position/global", 1, &FwNMPC::globPosCb, this);  //TODO: switch to UTM - stop doing own ll2ne
	odom_sub_			= nmpc_.subscribe("/mavros/global_position/local", 1, &FwNMPC::odomCb, this);
	ekf_ext_sub_		= nmpc_.subscribe("/mavros/aslekf_extended", 1, &FwNMPC::ekfExtCb, this);
	nmpc_params_sub_ 	= nmpc_.subscribe("/mavros/nmpc_params", 1, &FwNMPC::nmpcParamsCb, this);
	waypoint_list_sub_	= nmpc_.subscribe("/mavros/mission/waypoints", 1, &FwNMPC::waypointListCb, this);
	current_wp_sub_		= nmpc_.subscribe("/mavros/mission/current_wp", 1, &FwNMPC::currentWpCb, this);
	home_wp_sub_		= nmpc_.subscribe("/mavros/home_position", 1, &FwNMPC::homeWpCb, this);
	aslctrl_debug_sub_	= nmpc_.subscribe("/mavros/aslctrl/debug", 1, &FwNMPC::aslctrlDebugCb, this);

	/* publishers */
	obctrl_pub_ = nmpc_.advertise<mavros_msgs::AslObCtrl>("/nmpc/asl_obctrl", 10, true);
	nmpc_info_pub_ = nmpc_.advertise<fw_ctrl::NmpcInfo>("/nmpc/info",10,true);
	acado_vars_pub_	= nmpc_.advertise<fw_ctrl::AcadoVars>("/nmpc/acado_vars",10,true);
}

void FwNMPC::aslctrlDataCb(const mavros_msgs::AslCtrlData::ConstPtr& msg) {

	subs_.aslctrl_data.header		= msg->header;
	subs_.aslctrl_data.aslctrl_mode	= msg->aslctrl_mode;
	if (FAKE_SIGNALS==1) {
		double y_uT_ref;
		nmpc_.getParam("/nmpc/y_ref/uT", y_uT_ref);
		double y_theta_ref;
		nmpc_.getParam("/nmpc/y_ref/theta_ref", y_theta_ref);

		subs_.aslctrl_data.rollAngle	= 0.0;
		subs_.aslctrl_data.pitchAngle	= y_theta_ref;
		subs_.aslctrl_data.p 			= 0.0;
		subs_.aslctrl_data.q 			= 0.0;
		subs_.aslctrl_data.r 			= 0.0;
		subs_.aslctrl_data.airspeedRef	= 13.5;
		subs_.aslctrl_data.uThrot 		= y_uT_ref+CTRL_DEADZONE[ 0 ];
	}
	else if (FAKE_SIGNALS==2) {
		double y_uT_ref;
		nmpc_.getParam("/nmpc/y_ref/uT", y_uT_ref);
		double y_theta_ref;
		nmpc_.getParam("/nmpc/y_ref/theta_ref", y_theta_ref);

		subs_.aslctrl_data.rollAngle	= 0.0 + 0.05*(((double) rand() / (RAND_MAX)) * 2.0 - 1.0);
		subs_.aslctrl_data.pitchAngle	= y_theta_ref + 0.1*fabs(y_theta_ref)*(((double) rand() / (RAND_MAX)) * 2.0 - 1.0);
		subs_.aslctrl_data.p 			= 0.0;
		subs_.aslctrl_data.q 			= 0.0;
		subs_.aslctrl_data.r 			= 0.0;
		subs_.aslctrl_data.airspeedRef	= 13.5  + 0.2*fabs(13.5)*(((double) rand() / (RAND_MAX)) * 2.0 - 1.0);
		subs_.aslctrl_data.uThrot 		= y_uT_ref+CTRL_DEADZONE[ 0 ];
	}
	else {
		subs_.aslctrl_data.rollAngle	= msg->rollAngle * 0.017453292519943f;
		subs_.aslctrl_data.pitchAngle	= msg->pitchAngle * 0.017453292519943f;
		subs_.aslctrl_data.p 			= msg->p;
		subs_.aslctrl_data.q 			= msg->q;
		subs_.aslctrl_data.r 			= msg->r;
		subs_.aslctrl_data.airspeedRef	= msg->airspeedRef;
		subs_.aslctrl_data.uThrot 		= msg->uThrot;
	}

	float tmp_yaw = msg->yawAngle * 0.017453292519943f;

	if (!bYawReceived) {
		subs_.aslctrl_data.yawAngle = tmp_yaw;
		last_yaw_msg_ = tmp_yaw;
		bYawReceived = true;
		return;
	}

	float delta_yaw = tmp_yaw - last_yaw_msg_;
	if ( delta_yaw < -3.141592653589793 ) delta_yaw = delta_yaw + 6.283185307179586;
	if ( delta_yaw > 3.141592653589793 ) delta_yaw = delta_yaw - 6.283185307179586;

	subs_.aslctrl_data.yawAngle = subs_.aslctrl_data.yawAngle + delta_yaw;

	last_yaw_msg_ = tmp_yaw;
}

void FwNMPC::globPosCb(const sensor_msgs::NavSatFix::ConstPtr& msg) {

	subs_.glob_pos.latitude = msg->latitude;
	subs_.glob_pos.longitude = msg->longitude;
	subs_.glob_pos.altitude = msg->altitude;
}

void FwNMPC::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {

	if (FAKE_SIGNALS==1 || FAKE_SIGNALS==2) {
		subs_.odom.twist.twist.linear.x = 13.5*cos(subs_.aslctrl_data.yawAngle) + 4.0*cos(1.047); //vn
		subs_.odom.twist.twist.linear.y = 13.5*sin(subs_.aslctrl_data.yawAngle) + 4.0*sin(1.047); //ve
		subs_.odom.twist.twist.linear.z = 0.0 - 1.0; //vd
	}
	else {
		subs_.odom.twist.twist.linear.x = msg->twist.twist.linear.x; //vn
		subs_.odom.twist.twist.linear.y = msg->twist.twist.linear.y; //ve
		subs_.odom.twist.twist.linear.z = msg->twist.twist.linear.z; //vd
	}
}

void FwNMPC::ekfExtCb(const mavros_msgs::AslEkfExt::ConstPtr& msg) {

	if (FAKE_SIGNALS==1) {
		subs_.ekf_ext.airspeed 		= 13.5;
		subs_.ekf_ext.windSpeed		= 4.0;
		subs_.ekf_ext.windDirection = 1.047;
		subs_.ekf_ext.windZ			= -1.0;
	}
	else if (FAKE_SIGNALS==2) {
		subs_.ekf_ext.airspeed 		= 13.5;
		subs_.ekf_ext.windSpeed		= 4.0 + 0.2*4.0*(((double) rand() / (RAND_MAX)) * 2.0 - 1.0);
		subs_.ekf_ext.windDirection = 1.047 + 0.3*(((double) rand() / (RAND_MAX)) * 2.0 - 1.0);
		subs_.ekf_ext.windZ			= -1.0;
	}
	else {
		subs_.ekf_ext.airspeed 		= (msg->airspeed<1.0) ? 1.0 : msg->airspeed;
		subs_.ekf_ext.windSpeed		= msg->windSpeed;
		subs_.ekf_ext.windDirection = msg->windDirection;
		subs_.ekf_ext.windZ			= msg->windZ;
	}
}

void FwNMPC::nmpcParamsCb(const mavros_msgs::AslNmpcParams::ConstPtr& msg) {

	subs_.nmpc_params.R_acpt 			= msg->R_acpt;
	subs_.nmpc_params.ceta_acpt 		= msg->ceta_acpt;
	subs_.nmpc_params.alpha_p_co 		= msg->alpha_p_co;
	subs_.nmpc_params.alpha_m_co 		= msg->alpha_m_co;
	subs_.nmpc_params.alpha_delta_co 	= msg->alpha_delta_co;
	subs_.nmpc_params.T_b_lat 			= msg->T_b_lat;
	subs_.nmpc_params.T_b_lon 			= msg->T_b_lon;
	subs_.nmpc_params.Qdiag 			= msg->Qdiag;
}

void FwNMPC::waypointListCb(const mavros_msgs::WaypointList::ConstPtr& msg) {

	subs_.waypoint_list.waypoints = msg->waypoints;
}

void FwNMPC::currentWpCb(const std_msgs::Int32::ConstPtr& msg) {

	subs_.current_wp.data = msg->data;
}

void FwNMPC::homeWpCb(const mavros_msgs::HomePosition::ConstPtr& msg) {

	subs_.home_wp.latitude 	= (double)msg->latitude;
	subs_.home_wp.longitude = (double)msg->longitude;
	subs_.home_wp.altitude 	= (double)msg->altitude;
}

void FwNMPC::aslctrlDebugCb(const mavros_msgs::AslCtrlDebug::ConstPtr& msg) {

	obctrl_en_ = msg->i8_1;
}

int FwNMPC::initNMPC() {

	/* get loop rate */
	nmpc_.getParam("/nmpc/loop_rate", LOOP_RATE);

	/* get model discretization step */
	nmpc_.getParam("/nmpc/time_step", TSTEP);

	/* fake signals */
	nmpc_.getParam("/nmpc/fake_signals", FAKE_SIGNALS);

	/* Initialize ACADO variables */
	initACADOVars();
	ROS_ERROR("initNMPC: ACADO variables initialized");

	/* Initialize the solver. */
	int RET = initializeSolver();

	return RET;
}

void FwNMPC::initACADOVars() {

	//TODO: maybe actually wait for all subscriptions to be filled here before initializing?

	double y_uT_ref;
	nmpc_.getParam("/nmpc/y_ref/uT", y_uT_ref);
	double y_theta_ref;
	nmpc_.getParam("/nmpc/y_ref/theta_ref", y_theta_ref);

	/* put something reasonable here.. NOT all zeros, solver is initialized from here */
	double X[NX] 	= {0.0, 0.0, 0.0, 13.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, y_uT_ref, 0.0};
	double U[NU] 	= {y_uT_ref, 0.0, y_theta_ref};
	double OD[NOD]	= {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 30.0, 0.8, 0.0, 0.0, 0.0, 0.1396, -0.0524, 0.0349, 1.0, 2.0};

	double Y[NY] = {0.0, 0.0, 13.5, 0.0, 0.0, 0.0, 0.0, 0.0, y_uT_ref, 0.0, y_theta_ref};
	double W[NY] = {200.0, 200.0, 10.0, 20.0, 20.0, 5.0, 100.0, 40.0, 30.0, 50.0, 50.0};

	nmpc_.getParam("nmpc/w_scale/q1", W_scale_[0]);
	nmpc_.getParam("nmpc/w_scale/q2", W_scale_[1]);
	nmpc_.getParam("nmpc/w_scale/q3", W_scale_[2]);
	nmpc_.getParam("nmpc/w_scale/q4", W_scale_[3]);
	nmpc_.getParam("nmpc/w_scale/q5", W_scale_[4]);
	nmpc_.getParam("nmpc/w_scale/q6", W_scale_[5]);
	nmpc_.getParam("nmpc/w_scale/q7", W_scale_[6]);
	nmpc_.getParam("nmpc/w_scale/q8", W_scale_[7]);
	nmpc_.getParam("nmpc/w_scale/q9", W_scale_[8]);
	nmpc_.getParam("nmpc/w_scale/q10", W_scale_[9]);
	nmpc_.getParam("nmpc/w_scale/q11", W_scale_[10]);

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
		for (int j = 0; j < NOD; ++j) acadoVariables.od[ i * NOD + j ] = OD[ j ];
	}

	// references
	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NY; ++j) acadoVariables.y[ i * NY + j ] = Y[ j ];
	}

	// weights
	memset(acadoVariables.W, 0, sizeof(acadoVariables.W)); // fill all with zero
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < NY; ++j) {
			acadoVariables.W[ NY * NY * i + NY * j + j ] = W[ j ]  / W_scale_[ j ]; // fill diagonals
		}
	}
	memset(acadoVariables.WN, 0, sizeof(acadoVariables.WN)); // fill all with zero
	for (int i = 0; i < NYN; ++i) {
		acadoVariables.WN[ i * NYN + i ] =  W[ i ] / W_scale_[ i ]; // fill diagonals
	}
}

void FwNMPC::initHorizon() {

	ROS_ERROR("initHorizon: hold states constant through horizon (first time in loop)");

	//TODO: sliding window and/or low pass filter
	
	/* get current controls */
	double U[NU] = {(double)subs_.aslctrl_data.uThrot,0.0,0.0}; // TODO: could potentially pull all current refs.

	/* apply dead-zone */
//	for (int i = 0; i < NU; ++i)  {
//		if (fabs(U[ i ]) < CTRL_DEADZONE[ i ]) U[ i ] = 0.0; // in dead-zone : set to zero
//		else if (U[ i ] < 0.0) U[ i ] = U[ i ] + CTRL_DEADZONE[ i ]; // negative control value : add dead-zone threshold
//		else U[ i ] = U[ i ] - CTRL_DEADZONE[ i ]; // positive control value : subtract dead-zone threshold
//	}
	if (fabs(U[ 0 ]) < CTRL_DEADZONE[ 0 ]) U[ 0 ] = 0.0; // in dead-zone : set to zero
	else if (U[ 0 ] < 0.0) U[ 0 ] = U[ 0 ] + CTRL_DEADZONE[ 0 ]; // negative control value : add dead-zone threshold
	else U[ 0 ] = U[ 0 ] - CTRL_DEADZONE[ 0 ]; // positive control value : subtract dead-zone threshold

	/* normalize for internal model */
	for (int i = 0; i < NU; ++i)  U[ i ] = U[ i ] / CTRL_NORMALIZATION[ i ];

	/* saturate controls for internal model constraints */
	for (int i = 0; i < NU; ++i) {
		if (U[i] < CTRL_SATURATION[i][0]) U[i] = CTRL_SATURATION[i][0];
		if (U[i] > CTRL_SATURATION[i][1]) U[i] = CTRL_SATURATION[i][1];
	}

	/* hold current controls constant through horizon */
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < NU; ++j) acadoVariables.u[ i * NU + j ] = U[ j ];
	}

	double X0[NX];
	paths_.ll2NE(X0[0], X0[1], (double)subs_.glob_pos.latitude, (double)subs_.glob_pos.longitude); // n, e
	X0[2]	= -((double)subs_.glob_pos.altitude - paths_.getHomeAlt()); // d
	X0[3]	= (double)subs_.ekf_ext.airspeed; // V
	X0[4]	= -asin((double)((subs_.odom.twist.twist.linear.z - subs_.ekf_ext.windZ) / ((subs_.ekf_ext.airspeed<1.0) ? 1.0 : subs_.ekf_ext.airspeed))); // gamma
	X0[5]	= (double)subs_.aslctrl_data.yawAngle; // xi
	X0[6]	= (double)subs_.aslctrl_data.rollAngle; // phi
	X0[7]	= (double)subs_.aslctrl_data.pitchAngle; // theta
	X0[8]	= (double)subs_.aslctrl_data.p; // p
	X0[9]	= (double)subs_.aslctrl_data.q; // q
	X0[10]	= (double)subs_.aslctrl_data.r; // r
	X0[11]	= U[0]; // throt
	X0[12]	= 0.0; // xsw

	for (int i = 0; i < NX; ++i) acadoVariables.x0[ i ] = X0[ i ];

	/* hold current states constant through horizion */
	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NX-NX_AUGM; ++j) {
			if (j==0) {
				acadoVariables.x[ i * NX + j ] = X0[0] + subs_.odom.twist.twist.linear.x * getTimeStep() * (double)i; // linear propagation
			}
			else if (j==1) {
				acadoVariables.x[ i * NX + j ] = X0[1] + subs_.odom.twist.twist.linear.y * getTimeStep() * (double)i; // linear propagation
			}
			else if (j==2) {
				acadoVariables.x[ i * NX + j ] = X0[2] + subs_.odom.twist.twist.linear.z * getTimeStep() * (double)i; // linear propagation
			}
			else if (j==8 || j==9 || j==10) {
				acadoVariables.x[ i * NX + j ] = 0.0; //zero out rates in horizon to avoid blow-up initializations
			}
			else {
				acadoVariables.x[ i * NX + j ] = acadoVariables.x0[ j ];
			}
		}
	}
}

void FwNMPC::updateACADO_X0() {

	double X0[NX];
	paths_.ll2NE(X0[0], X0[1], (double)subs_.glob_pos.latitude, (double)subs_.glob_pos.longitude); 				// n, e
	X0[2]	= -((double)subs_.glob_pos.altitude - paths_.getHomeAlt()); 										// d
	X0[3]	= (double)subs_.ekf_ext.airspeed; 																	// V
	X0[4]	= -asin((double)((subs_.odom.twist.twist.linear.z - subs_.ekf_ext.windZ) / ((subs_.ekf_ext.airspeed<1.0) ? 1.0 : subs_.ekf_ext.airspeed))); 		// gamma
	X0[5]	= (double)subs_.aslctrl_data.yawAngle; 																// xi
	X0[6]	= (double)subs_.aslctrl_data.rollAngle; 															// phi
	X0[7]	= (double)subs_.aslctrl_data.pitchAngle; 															// theta
	X0[8]	= (double)subs_.aslctrl_data.p; 																	// p
	X0[9]	= (double)subs_.aslctrl_data.q; 																	// q
	X0[10]	= (double)subs_.aslctrl_data.r; 																	// r
	if (FAKE_SIGNALS==1) {
		// no shifting internal states when faking signals
		X0[11]	= acadoVariables.x[11];
		X0[12]	= acadoVariables.x[12];
	}
	else {
		// NOTE: yes.. this is shifting the state. Only for these internal states, however.--linear interpolation
		X0[11]	= acadoVariables.x[11]+(acadoVariables.x[NX+11]-acadoVariables.x[11])/getLoopRate()/getTimeStep(); 	// NEXT throt state in horizon.
		X0[12]	= acadoVariables.x[12]+(acadoVariables.x[NX+12]-acadoVariables.x[12])/getLoopRate()/getTimeStep(); 	// NEXT xsw state in horizon.
	}
	bool fake_motor_failure=false;
	nmpc_.getParam("/nmpc/fake_motor_failure", fake_motor_failure);
	if (fake_motor_failure) X0[12]=0.0;

	for (int i = 0; i < NX; ++i) acadoVariables.x0[ i ] = X0[ i ];

	/* calculate track error for monitoring on QGC */
	double temp_in[NX+NOD];
	for (int i = 0; i < NX; ++i) temp_in[ i ] = acadoVariables.x0[ i ];
	for (int i = NX; i < NX+NOD; ++i) temp_in[ i ] = acadoVariables.od[ i - NX ];
	calculateTrackError(temp_in);
}

void FwNMPC::updateACADO_OD() {

	/* update online data */
	double OD[NOD];
	OD[0] = paths_.path_current.pparam1;
	OD[1] = paths_.path_current.pparam2;
	OD[2] = paths_.path_current.pparam3;
	OD[3] = paths_.path_current.pparam4;
	OD[4] = paths_.path_current.pparam5;
	OD[5] = paths_.path_current.pparam6;
	OD[6] = paths_.path_current.pparam7;
	OD[7] = paths_.path_current.pparam8;
	OD[8] = paths_.path_current.pparam9;
	OD[9] = paths_.path_next.pparam1;
	OD[10] = paths_.path_next.pparam2;
	OD[11] = paths_.path_next.pparam3;
	OD[12] = paths_.path_next.pparam4;
	OD[13] = paths_.path_next.pparam5;
	OD[14] = paths_.path_next.pparam6;
	OD[15] = paths_.path_next.pparam7;
	OD[16] = paths_.path_next.pparam8;
	OD[17] = paths_.path_next.pparam9;
	OD[18] = (double)subs_.nmpc_params.R_acpt;
	OD[19] = (double)subs_.nmpc_params.ceta_acpt;
	OD[20] = (double)subs_.ekf_ext.windSpeed * cos((double)subs_.ekf_ext.windDirection);
	OD[21] = (double)subs_.ekf_ext.windSpeed * sin((double)subs_.ekf_ext.windDirection);
	OD[22] = (double)subs_.ekf_ext.windZ;
	OD[23] = (double)subs_.nmpc_params.alpha_p_co;
	OD[24] = (double)subs_.nmpc_params.alpha_m_co;
	OD[25] = (double)subs_.nmpc_params.alpha_delta_co;
	OD[26] = (double)subs_.nmpc_params.T_b_lat;
	OD[27] = (double)subs_.nmpc_params.T_b_lon;

	for (int i = 0; i < N+1; ++i) {
			for (int j = 0; j < NOD; ++j) {
					acadoVariables.od[ i * NOD + j ] = OD[ j ];
			}
	}
}

void FwNMPC::updateACADO_Y() {

	double y_uT_ref;
	nmpc_.getParam("/nmpc/y_ref/uT", y_uT_ref);
	bool fake_motor_failure=false;
	nmpc_.getParam("/nmpc/fake_motor_failure", fake_motor_failure);
	if (fake_motor_failure) y_uT_ref=0.0;
	double y_theta_ref;
	nmpc_.getParam("/nmpc/y_ref/theta_ref", y_theta_ref);

	/* update references */
	double Y[NY];
	Y[0] 	= 0.0;								// eta_lat _ref
	Y[1] 	= 0.0;								// eta_lon _ref
	Y[2] 	= subs_.aslctrl_data.airspeedRef;	// V_ref
	Y[3] 	= 0.0; 								// p_ref
	Y[4] 	= 0.0;								// q_ref
	Y[5] 	= 0.0;								// r_ref
	Y[6] 	= 0.0;								// alpha_soft _ref
	Y[7] 	= 0.0;  							// delta_T_dot _ref
	Y[8] 	= y_uT_ref;							// uthrot _ref
	Y[9] 	= 0.0;								// phi_ref _ref
	Y[10] 	= y_theta_ref;						// theta_ref _ref

	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < NY; ++j) {
			acadoVariables.y[ i * NY + j ] = Y[ j ];
		}
	}
}

void FwNMPC::updateACADO_W() {

	/* update objective gains */
	double W[NY];
	for (int i = 0; i < NY; i++) W[ i ] = (double)subs_.nmpc_params.Qdiag[ i ];

	bool fake_motor_failure=false;
	nmpc_.getParam("/nmpc/fake_motor_failure", fake_motor_failure);
	if (fake_motor_failure) W[8]=1e6;

	// only update diagonal terms
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < NY; ++j) {
			acadoVariables.W[ NY * NY * i + NY * j + j ] = W[ j ]  / W_scale_[ j ];
		}
	}
	for (int i = 0; i < NYN; ++i) {
		acadoVariables.WN[ i * NYN + i ] =  W[ i ]  / W_scale_[ i ];
	}
}

double FwNMPC::getLoopRate() {
	return LOOP_RATE;
}

double FwNMPC::getTimeStep() {
	return TSTEP;
}

void FwNMPC::reqSubs() { //TODO: extend this and/or change this to all subs/initializations

	/* initialize home wp and current wp seq structs while waiting for real data */
	subs_.home_wp.latitude = 0.0;
	subs_.home_wp.longitude = 0.0;
	subs_.home_wp.altitude = 0.0;
	subs_.current_wp.data = 0;

	/* pull current waypoint list */
	ros::ServiceClient wp_pull_client_ = nmpc_.serviceClient<mavros_msgs::WaypointPull>("/mavros/mission/pull");
	mavros_msgs::WaypointPull wp_pull_srv;

	if (wp_pull_client_.call(wp_pull_srv)) {
		ROS_ERROR("reqSubs: received %d waypoints", (int)wp_pull_srv.response.wp_received);
	}
	else {
		ROS_ERROR("reqSubs: failed to call wp pull service");
	}

	/* wait for home waypoint */
	while ( subs_.home_wp.latitude < 0.1 && subs_.home_wp.longitude < 0.1 ) {

		ros::spinOnce();

		ROS_INFO ("fw_nmpc: waiting for home waypoint");

		sleep(0.5);
	}

	ROS_ERROR("reqSubs: received home waypoint");

	return;
}

void FwNMPC::calculateTrackError(const real_t *in) {
	// Code snippet from end lsq term in model.c

	/* for manual input indexing ... */

	const int minus_NU = ACADO_NU;

	/* optimized intermediate calculations */

	const double t2 = cos(in[4]);

	double Vsafe = in[3];
	if (Vsafe<1.0) Vsafe = 1.0;

	const double n_dot = in[33]+Vsafe*t2*cos(in[5]);
	const double e_dot = in[34]+Vsafe*t2*sin(in[5]);
	const double d_dot = in[35]-Vsafe*sin(in[4]);

	/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

	// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
	const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
	int pparam_sel = 0;

	double p_n = 0.0;
	double p_e = 0.0;
	double p_d = 0.0;
	double tP_n = 1.0;
	double tP_e = 0.0;
	double tP_d = 0.0;

	const double pparam_type = in[idx_OD_0+pparam_sel];

	// LINE SEGMENT
	if ( pparam_type < 0.5 ) {

	    // variable definitions
	    const double pparam_aa_n = in[idx_OD_0+pparam_sel+1];
	    const double pparam_aa_e = in[idx_OD_0+pparam_sel+2];
	    const double pparam_aa_d = in[idx_OD_0+pparam_sel+3];
	    const double pparam_bb_n = in[idx_OD_0+pparam_sel+4];
	    const double pparam_bb_e = in[idx_OD_0+pparam_sel+5];
	    const double pparam_bb_d = in[idx_OD_0+pparam_sel+6];

	    // calculate vector from waypoint a to b
	    const double abn = pparam_bb_n - pparam_aa_n;
	    const double abe = pparam_bb_e - pparam_aa_e;
	    const double abd = pparam_bb_d - pparam_aa_d;
	    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

	    // calculate tangent
	    if (norm_ab>0.1) {
	        tP_n = abn / norm_ab;
	        tP_e = abe / norm_ab;
	        tP_d = abd / norm_ab;
	    }
	    
	    // dot product
	    const double dot_abunit_ap = tP_n*(in[0] - pparam_aa_n) + tP_e*(in[1] - pparam_aa_e) + tP_d*(in[2] - pparam_aa_d);
	    
	    // point on track
	    p_n = pparam_aa_n + dot_abunit_ap * tP_n;
	    p_e = pparam_aa_e + dot_abunit_ap * tP_e;
	    p_d = pparam_aa_d + dot_abunit_ap * tP_d;
	    
	// CURVE SEGMENT
	} else if ( pparam_type < 1.5 ) {

	    // variable definitions
	    const double pparam_cc_n = in[idx_OD_0+pparam_sel+1];
	    const double pparam_cc_e = in[idx_OD_0+pparam_sel+2];
	    const double pparam_cc_d = in[idx_OD_0+pparam_sel+3];
	    const double pparam_R = in[idx_OD_0+pparam_sel+4];
	    const double pparam_ldir = in[idx_OD_0+pparam_sel+5];
	    const double pparam_gam_sp = in[idx_OD_0+pparam_sel+6];
	    const double pparam_xi0 = in[idx_OD_0+pparam_sel+7];
	    const double pparam_dxi = in[idx_OD_0+pparam_sel+8];

	    // calculate closest point on loiter circle
	    const double cp_n = in[0] - pparam_cc_n;
	    const double cp_e = in[1] - pparam_cc_e;
	    const double norm_cr = sqrt( cp_n*cp_n + cp_e*cp_e );
	    double cp_n_unit;
	    double cp_e_unit;
	    if (norm_cr>0.1) {
	        cp_n_unit = cp_n / norm_cr;
	        cp_e_unit = cp_e / norm_cr;
	    }
	    else {
	        cp_n_unit = 0.0;
	        cp_e_unit = 0.0;
	    }
	    p_n = pparam_R * cp_n_unit + pparam_cc_n;
	    p_e = pparam_R * cp_e_unit + pparam_cc_e;

	    // calculate tangent
	    tP_n = pparam_ldir * -cp_e_unit;
	    tP_e = pparam_ldir * cp_n_unit;
	    
	    // spiral angular position: [0,2*pi)
	    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
	    double delta_xi_p = xi_sp-pparam_xi0;
	    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

	        delta_xi_p = delta_xi_p + 6.28318530718;

	    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

	        delta_xi_p = delta_xi_p - 6.28318530718;

	    }

	    // closest point on nearest spiral leg and tangent down component
	    if (fabs(pparam_gam_sp) < 0.001) {

	        p_d = pparam_cc_d;
	        tP_d = 0.0;

	    } else {

	        const double Rtangam = pparam_R * tan(pparam_gam_sp);

	        // spiral height delta for current angle
	        const double delta_d_xi = -delta_xi_p * Rtangam;

	        // end spiral altitude change
	        const double delta_d_sp_end = -pparam_dxi * Rtangam;

	        // nearest spiral leg
	        const double delta_d_k = round( (in[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

	        // closest point on nearest spiral leg
	        p_d = pparam_cc_d + delta_d_k + delta_d_xi;

	        /* p (on spiral) = (start height) + (revolution height increment) +
	         * (lateral-direcitonal angular position increment)
	         */
	        
	        // cap end point
	        if ((p_d - (delta_d_sp_end + pparam_cc_d)) * pparam_gam_sp < 0.0) {
	            // we (or more correctly, the closest point on the nearest spiral leg) are beyond the end point
	            p_d = pparam_cc_d + delta_d_sp_end;
	            tP_d = 0.0;
	        }
	        else {
	            tP_d = -sin(pparam_gam_sp);
	        }
	        
	    }
	    
	    if (fabs(tP_n)<0.01 && fabs(tP_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
	        tP_n=1.0;
	    }
	    
	    // Renormalize tP
	    const double normtP = sqrt(tP_n*tP_n+tP_e*tP_e+tP_d*tP_d);
	    tP_n = tP_n / normtP;
	    tP_e = tP_e / normtP;
	    tP_d = tP_d / normtP;
	        
	}

	/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

	const double alpha = -in[4]+in[7];

	const double t3 = in[1]-p_e;
	const double t4 = in[0]-p_n;
	const double norm_rp_ne = sqrt(t3*t3+t4*t4);

	const double t5 = 1.0/norm_rp_ne;
	const double t6 = -in[2]+p_d;

	double sgn_rp = 0.0;
	if (t6>0.0) {
	    sgn_rp = 1.0;
	} else if (t6<0.0) {
	    sgn_rp = -1.0;
	}

	const double t16 = e_dot*e_dot;
	const double t17 = n_dot*n_dot;
	const double t18 = t16+t17;

	track_error_lat_ = t4*tP_e-t3*tP_n;
	track_error_lon_ = t6;
}

int FwNMPC::nmpcIteration() {

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

			// update home wp
			// -->initHorizon needs up to date wp info for ned calculation
			const double new_home_wp[3] = {subs_.home_wp.latitude, subs_.home_wp.longitude, subs_.home_wp.altitude};
			paths_.setHomeWp( new_home_wp );
			prev_wp_idx_ = -1;
			last_wp_idx_ = -1;

			// initHorizon BEFORE Y, this is to make sure controls and prev_horiz are reinitialized before reaching Y update.
			initHorizon();
			updateACADO_Y();
			updateACADO_W();

			bModeChanged = false;
		}
		else {
			//regular update

			/* update ACADO states/references/weights */
			updateACADO_X0(); // note this shifts augmented states
			updateACADO_Y();
			updateACADO_W();
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

			// set previous wp list item to last wp id
			prev_wp_idx_ = (last_wp_idx_==-1) ? subs_.current_wp.data : last_wp_idx_; // if first time through loop, previous wp will be set to current wp
		}
		last_wp_idx_ = subs_.current_wp.data;

		/* update path */
		paths_.updatePaths(subs_.waypoint_list, subs_.current_wp.data, prev_wp_idx_);

		/* update ACADO online data */
		updateACADO_OD();

		/* waypoint management time in us <-- */
		t_elapsed = ros::Time::now() - t_wp_man_start;
		t_wp_man = t_elapsed.toNSec()/1000;

		/* start nmpc solve timer --> */
		ros::Time t_solve_start = ros::Time::now();

		/* Prepare first step */
		RET[0] = preparationStep();
		if ( RET[0] != 0 ) {
			ROS_ERROR("nmpcIteration: preparation step error, code %d", RET[0]);
			obctrl_status = RET[0];
		}

		/* Perform the feedback step. */
		RET[1] = feedbackStep();
		if ( RET[1] != 0 ) {
			ROS_ERROR("nmpcIteration: feedback step error, code %d", RET[1]);
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

void FwNMPC::publishControls(uint64_t &t_ctrl, uint64_t t_iter_approx, int obctrl_status) {

	//TODO: this should be interpolated in the event of Tnmpc ~= Tfcall
	/* Apply the control immediately to the process, first NU components. */

	double ctrl[NU];
	for (int i = 0; i < NU; ++i)  ctrl[ i ] = acadoVariables.u[ i ];

	/* saturate controls for internal model constraints */
	for (int i = 0; i < NU; ++i) {
		if (ctrl[i] < CTRL_SATURATION[i][0]) ctrl[i] = CTRL_SATURATION[i][0];
		if (ctrl[i] > CTRL_SATURATION[i][1]) ctrl[i] = CTRL_SATURATION[i][1];
	}

	/* de-normalize */
	for (int i = 0; i < NU; ++i)  ctrl[ i ] = ctrl[ i ] * CTRL_NORMALIZATION[ i ];

	/* re-apply dead-zone offset */
//	for (int i = 0; i < NU; ++i)  {
//		if (ctrl[ i ] < 0.02 * CTRL_NORMALIZATION[ i ] * CTRL_SATURATION[i][0]) ctrl[ i ] = ctrl[ i ] - CTRL_DEADZONE[ i ];
//		else if (ctrl[ i ] > 0.02 * CTRL_NORMALIZATION[ i ] * CTRL_SATURATION[i][1]) ctrl[ i ] = ctrl[ i ] + CTRL_DEADZONE[ i ];
//		else ctrl[ i ] = 0.0;
//	}
	if (ctrl[ 0 ] < 0.02 * CTRL_NORMALIZATION[ 0 ] * CTRL_SATURATION[0][0]) ctrl[ 0 ] = ctrl[ 0 ] - CTRL_DEADZONE[ 0 ];
	else if (ctrl[ 0 ] > 0.02 * CTRL_NORMALIZATION[ 0 ] * CTRL_SATURATION[0][1]) ctrl[ 0 ] = ctrl[ 0 ] + CTRL_DEADZONE[ 0 ];
	else ctrl[ 0 ] = 0.0;

	/* publish obctrl msg */
	mavros_msgs::AslObCtrl obctrl_msg;
	obctrl_msg.timestamp = t_iter_approx;
	obctrl_msg.uThrot = ( isnan((float)ctrl[0]) ) ? 0.0f : (float)ctrl[0];
	obctrl_msg.uThrot2 = (float)track_error_lon_; // for monitoring on QGC
	obctrl_msg.uAilR = ( isnan((float)ctrl[1]) ) ? 0.0f : (float)ctrl[1];
	obctrl_msg.uAilL = (float)track_error_lat_; // for monitoring on QGC
	obctrl_msg.uElev = ( isnan((float)ctrl[2]) ) ? 0.0f : (float)ctrl[2];
	obctrl_msg.obctrl_status = ( isnan((float)ctrl[0]) || isnan((float)ctrl[1]) || isnan((float)ctrl[2]) ) ? 11 : (uint8_t)obctrl_status; // status=11 for nan detection

	obctrl_pub_.publish(obctrl_msg);

	/* update last control timestamp / publish elapsed ctrl loop time */
	ros::Duration t_elapsed = ros::Time::now() - t_lastctrl;
	t_ctrl = t_elapsed.toNSec()/1000; //us
	t_lastctrl = ros::Time::now();
}

void FwNMPC::publishAcadoVars() {

	fw_ctrl::AcadoVars acado_vars;

	/* measurements */
	for (int i=0; i<NX; i++) {
		acado_vars.x0[i] = (float)acadoVariables.x0[i];
	}

	/* state horizons */
	for (int i=0; i<N+1; i++) {
		acado_vars.n[i] = (float)acadoVariables.x[NX * i];
		acado_vars.e[i] = (float)acadoVariables.x[NX * i + 1];
		acado_vars.d[i] = (float)acadoVariables.x[NX * i + 2];
		acado_vars.V[i] = (float)acadoVariables.x[NX * i + 3];
		acado_vars.gamma[i] = (float)acadoVariables.x[NX * i + 4];
		acado_vars.xi[i] = (float)acadoVariables.x[NX * i + 5];
		acado_vars.phi[i] = (float)acadoVariables.x[NX * i + 6];
		acado_vars.theta[i] = (float)acadoVariables.x[NX * i + 7];
		acado_vars.p[i] = (float)acadoVariables.x[NX * i + 8];
		acado_vars.q[i] = (float)acadoVariables.x[NX * i + 9];
		acado_vars.r[i] = (float)acadoVariables.x[NX * i + 10];
		acado_vars.throt[i] = (float)acadoVariables.x[NX * i + 11];
		acado_vars.xsw[i] = (float)acadoVariables.x[NX * i + 12];
	}

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
	acado_vars.pparam8 = (float)acadoVariables.od[7];
	acado_vars.pparam9 = (float)acadoVariables.od[8];
	acado_vars.pparam1_next = (uint8_t)acadoVariables.od[9];
	acado_vars.pparam2_next = (float)acadoVariables.od[10];
	acado_vars.pparam3_next = (float)acadoVariables.od[11];
	acado_vars.pparam4_next = (float)acadoVariables.od[12];
	acado_vars.pparam5_next = (float)acadoVariables.od[13];
	acado_vars.pparam6_next = (float)acadoVariables.od[14];
	acado_vars.pparam7_next = (float)acadoVariables.od[15];
	acado_vars.pparam8_next = (float)acadoVariables.od[16];
	acado_vars.pparam9_next = (float)acadoVariables.od[17];
	acado_vars.R_acpt = (float)acadoVariables.od[18];
	acado_vars.ceta_acpt = (float)acadoVariables.od[19];
	acado_vars.wn = (float)acadoVariables.od[20];
	acado_vars.we = (float)acadoVariables.od[21];
	acado_vars.wd = (float)acadoVariables.od[22];
	acado_vars.alpha_p_co = (float)acadoVariables.od[23];
	acado_vars.alpha_m_co = (float)acadoVariables.od[24];
	acado_vars.alpha_delta_co = (float)acadoVariables.od[25];
	acado_vars.T_b_lat = (float)acadoVariables.od[26];
	acado_vars.T_b_lon = (float)acadoVariables.od[27];

	/* references */ //NOTE: only recording non-zero references
//	acado_vars.y_eta_lat = (float)acadoVariables.y[0];
//	acado_vars.y_eta_lon = (float)acadoVariables.y[1];
	acado_vars.y_V = (float)acadoVariables.y[2];
//	acado_vars.y_p = (float)acadoVariables.y[3];
//	acado_vars.y_q = (float)acadoVariables.y[4];
//	acado_vars.y_r = (float)acadoVariables.y[5];
//	acado_vars.y_asoft = (float)acadoVariables.y[6];
//	acado_vars.y_uTdot = (float)acadoVariables.y[7];
	acado_vars.y_uT = (float)acadoVariables.y[8];
//	acado_vars.y_phi_ref = (float)acadoVariables.y[9];
	acado_vars.y_theta_ref = (float)acadoVariables.y[10];

	acado_vars_pub_.publish(acado_vars);
}

void FwNMPC::publishNmpcInfo(ros::Time t_iter_start, uint64_t t_ctrl, uint64_t t_solve, uint64_t t_update, uint64_t t_wp_man) {

	fw_ctrl::NmpcInfo nmpc_info;

	/* solver info */
	nmpc_info.kkt = (double)getKKT();
	nmpc_info.obj = (double)getObjective();

	/* various elapsed times */
	nmpc_info.t_ctrl = t_ctrl;		// time elapsed since last control action was published (stays zero if not in auto mode)
	nmpc_info.t_solve = t_solve;	// time elapsed during nmpc preparation and feedback step (solve time)
	nmpc_info.t_update = t_update;	// time elapsed during array updates
	nmpc_info.t_wp_man = t_wp_man;	// time elapsed during waypoint management

	/* nmpc iteration time in us */
	ros::Duration t_elapsed = ros::Time::now() - t_iter_start;
	nmpc_info.t_iter = (uint64_t)(t_elapsed.toNSec()/1000);

	nmpc_info_pub_.publish(nmpc_info);
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
	ROS_ERROR("fw_nmpc: entering NMPC loop");
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

	ROS_ERROR("fw_nmpc: closing...");

	return 0;
}
