#include <ros/ros.h>
#include <ros/package.h>

#include <fw_nmpc.h>

#include <math.h>
#include <string.h>

#include <std_msgs/UInt8.h>

// INCLUDES for mavros
#include <mavros/WaypointPull.h>

// INCLUDES for fw_ctrl
#include <fw_ctrl/AcadoVars.h>
#include <fw_ctrl/NmpcInfo.h>
#include <mavros/AslObCtrl.h>

using namespace fw_nmpc;

FwNMPC::FwNMPC() :
		LOOP_RATE(20.0), //MAGIC NUMBER
		TSTEP(0.1), //MAGIC NUMBER
		t_lastctrl{0},
		bModeChanged(false),
		last_ctrl_mode(0),
		prev_ctrl_horiz_{0},
		prev_wp_idx_(-1),
		last_wp_idx_(-1),
		bYawReceived(false),
		last_yaw_msg_(0.0f)
{

	ROS_INFO("Instance of NMPC created");

	/* subscribers */
	aslctrl_data_sub_	= nmpc_.subscribe("/mavros/aslctrl/data", 1, &FwNMPC::aslctrlDataCb, this);
	glob_pos_sub_ 		= nmpc_.subscribe("/mavros/global_position/global", 1, &FwNMPC::globPosCb, this);
	glob_vel_sub_ 		= nmpc_.subscribe("/mavros/global_position/gp_vel", 1, &FwNMPC::globVelCb, this);
	ekf_ext_sub_			= nmpc_.subscribe("/mavros/aslekf_extended", 1, &FwNMPC::ekfExtCb, this);
	nmpc_params_sub_ 	= nmpc_.subscribe("/mavros/nmpc_params", 1, &FwNMPC::nmpcParamsCb, this);
	waypoint_list_sub_= nmpc_.subscribe("/mavros/mission/waypoints", 1, &FwNMPC::waypointListCb, this);
	current_wp_sub_		= nmpc_.subscribe("/mavros/mission/current_wp", 1, &FwNMPC::currentWpCb, this);
	home_wp_sub_			= nmpc_.subscribe("/mavros/mission/home_wp", 1, &FwNMPC::homeWpCb, this);

	/* publishers */
	obctrl_pub_ = nmpc_.advertise<mavros::AslObCtrl>("/nmpc/asl_obctrl", 10, true);
	nmpc_info_pub_ = nmpc_.advertise<fw_ctrl::NmpcInfo>("/nmpc/info",10,true);
	acado_vars_pub_	= nmpc_.advertise<fw_ctrl::AcadoVars>("/nmpc/acado_vars",10,true);
}

void FwNMPC::aslctrlDataCb(const mavros::AslCtrlData::ConstPtr& msg) {

	subs_.aslctrl_data.header 				= msg->header;
	subs_.aslctrl_data.aslctrl_mode 	= msg->aslctrl_mode;
	subs_.aslctrl_data.RollAngle			= msg->RollAngle * 0.017453292519943f;
	subs_.aslctrl_data.PitchAngle			= msg->PitchAngle * 0.017453292519943f;
	subs_.aslctrl_data.p							= msg->p;
	subs_.aslctrl_data.q							= msg->q;
	subs_.aslctrl_data.r							= msg->r;
	subs_.aslctrl_data.AirspeedRef		= msg->AirspeedRef;

	float tmp_yaw = msg->YawAngle * 0.017453292519943f;

	if (!bYawReceived) {
		subs_.aslctrl_data.YawAngle = tmp_yaw;
		last_yaw_msg_ = tmp_yaw;
		bYawReceived = true;
		return;
	}

	float delta_yaw = tmp_yaw - last_yaw_msg_;
	if ( delta_yaw < -3.141592653589793 ) delta_yaw = delta_yaw + 6.283185307179586;
	if ( delta_yaw > 3.141592653589793 ) delta_yaw = delta_yaw - 6.283185307179586;

	subs_.aslctrl_data.YawAngle = subs_.aslctrl_data.YawAngle + delta_yaw;

	last_yaw_msg_ = tmp_yaw;
}

void FwNMPC::globPosCb(const sensor_msgs::NavSatFix::ConstPtr& msg) {

	subs_.glob_pos.latitude = msg->latitude;
	subs_.glob_pos.longitude = msg->longitude;
	subs_.glob_pos.altitude = msg->altitude;
}

void FwNMPC::globVelCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {

	subs_.glob_vel.vector.x = msg->vector.x; //vn
	subs_.glob_vel.vector.y = msg->vector.y; //ve
	subs_.glob_vel.vector.z = msg->vector.z; //vd
}

void FwNMPC::ekfExtCb(const mavros::AslEkfExt::ConstPtr& msg) {

	subs_.ekf_ext.airspeed 			= msg->airspeed;
	subs_.ekf_ext.windSpeed 		= msg->windSpeed;
	subs_.ekf_ext.windDirection = msg->windDirection;
	subs_.ekf_ext.windZ					= msg->windZ;
}

void FwNMPC::nmpcParamsCb(const mavros::AslNmpcParams::ConstPtr& msg) {

	subs_.nmpc_params.R_acpt = msg->R_acpt;
	subs_.nmpc_params.ceta_acpt = msg->ceta_acpt;
	subs_.nmpc_params.k_t_d = msg->k_t_d;
	subs_.nmpc_params.e_d_co = msg->e_d_co;
	subs_.nmpc_params.k_t_ne = msg->k_t_ne;
	subs_.nmpc_params.e_ne_co = msg->e_ne_co;
	subs_.nmpc_params.eps_v = msg->eps_v;
	subs_.nmpc_params.alpha_p_co = msg->alpha_p_co;
	subs_.nmpc_params.alpha_m_co = msg->alpha_m_co;
	subs_.nmpc_params.alpha_delta_co = msg->alpha_delta_co;
	subs_.nmpc_params.Qdiag = msg->Qdiag;
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
	ROS_ERROR("initNMPC: ACADO variables initialized");

	/* Initialize the solver. */
	int RET = initializeSolver();

	/* get loop rate */
	nmpc_.getParam("/nmpc/loop_rate", LOOP_RATE);

	/* get model discretization step */
	nmpc_.getParam("/nmpc/time_step", TSTEP);

	return RET;
}

void FwNMPC::initACADOVars() {

	//TODO: maybe actually wait for all subscriptions to be filled here before initializing?

	/* put something reasonable here.. NOT all zeros, solver is initialized from here */
	double X[NX] = {0.0, 0.0, 0.0, 13.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0};
	double U[NU] = {0.3, 0.0, 0.0};
	double OD[NOD] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 30.0, 0.8, 0.0, 0.0, 0.0, 0.4, 30.0, 0.4, 60.0, 0.5, 0.1396, -0.0524, 0.0349};
	double Y[NY] = {0.0, 0.0, 0.0, 0.0, 0.0, 13.5, 0.0, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.3, 0.0, 0.0};
	double W[NY] = {50.0, 200.0, 20.0, 20.0, 5.0, 10.0, 36.47, 36.47, 3.647, 1.0, 0.1, 3.6476, 1.459, 10.0, 182.3, 145.9};

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

	// weights -- fill all non-diagonals with zero!
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < NY; ++j) {
			for (int k = 0; k < NY; ++k) {
				if ( j == k ) {
//					if ( j<NY-NU+1 ) { //NOTE: includes constant weight through horizon for previous throttle
						acadoVariables.W[ NY * NY * i + NY * j + j ] = W[ j ];
//					}
//					else {
//						acadoVariables.W[ NY * NY * i + NY * j + j ] = W[ j ] * (1.0 - i / N) * (1.0 - i / N); //NOTE: for phi_ref and theta_ref, reduce weight on previous value through horizon
//					}
				}
				else {
					acadoVariables.W[ NY * NY * i + NY * j + k ] = 0.0;
				}
			}
		}
	}
	for (int i = 0; i < NYN; ++i) {
		for (int j = 0; j < NYN; ++j) {
			if ( i == j ) {
//				if ( i<NYN-1 ) {
					acadoVariables.WN[ i * NYN + i ] =  W[ i ];
//				}
//				else {
//					acadoVariables.WN[ i * NYN + i ] =  0.0;
//				}
			}
			else {
				acadoVariables.WN[ i * NYN + j ] =  0.0;
			}
		}
	}
}

void FwNMPC::initHorizon() {

	ROS_ERROR("initHorizon: hold states constant through horizon (first time in loop)");

	//TODO: sliding window and/or low pass filter

	double X0[NX];
	paths_.ll2NE(X0[0], X0[1], (double)subs_.glob_pos.latitude, (double)subs_.glob_pos.longitude); // n, e
	X0[2]		= -((double)subs_.glob_pos.altitude - paths_.getHomeAlt()); // d
	X0[3]		= (double)subs_.ekf_ext.airspeed; // V
	X0[4]		= (double)( (subs_.glob_vel.vector.z - subs_.ekf_ext.windZ) / subs_.ekf_ext.airspeed ); // gamma
	X0[5]		= (double)subs_.aslctrl_data.YawAngle; // xi
	X0[6]		= (double)subs_.aslctrl_data.RollAngle; // phi
	X0[7]		= (double)subs_.aslctrl_data.PitchAngle; // theta
	X0[8]		= (double)subs_.aslctrl_data.p; // p
	X0[9]		= (double)subs_.aslctrl_data.q; // q
	X0[10]	= (double)subs_.aslctrl_data.r; // r
	X0[11]	= (double)subs_.aslctrl_data.uThrot; // throt
	X0[12]	= 0.0; // xsw

	for (int i = 0; i < NX; ++i) acadoVariables.x0[ i ] = X0[ i ];

	/* hold current states constant through horizion */
	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NX-NX_AUGM; ++j) {
			if (j==0) {
				acadoVariables.x[ i * NX + j ] = X0[0] + subs_.glob_vel.vector.x * getTimeStep() * (double)i; // linear propagation
			}
			else if (j==1) {
				acadoVariables.x[ i * NX + j ] = X0[1] + subs_.glob_vel.vector.y * getTimeStep() * (double)i; // linear propagation
			}
			else if (j==2) {
				acadoVariables.x[ i * NX + j ] = X0[2] + subs_.glob_vel.vector.z * getTimeStep() * (double)i; // linear propagation
			}
			else if (j==8 || j==9 || j==10) {
				acadoVariables.x[ i * NX + j ] = 0.0; //zero out rates in horizon to avoid blow-up initializations
			}
			else {
				acadoVariables.x[ i * NX + j ] = acadoVariables.x0[ j ];
			}
		}
	}

	/* get current controls */
	double U[NU] = {(double)subs_.aslctrl_data.uThrot,0.0,0.0}; // could potentially pull all current refs.

	/* offset controls */
	for (int i = 0; i < NU; ++i)  U[ i ] = U[ i ] - subs_.CTRL_OFFSET[ i ];

	/* saturate controls */
	for (int i = 0; i < NU; ++i) {
		if (U[i] < subs_.CTRL_SATURATION[i][0]) U[i] = subs_.CTRL_SATURATION[i][0];
		if (U[i] > subs_.CTRL_SATURATION[i][1]) U[i] = subs_.CTRL_SATURATION[i][1];
	}

	/* normalize controls */
	for (int i = 0; i < NU; ++i)  U[ i ] = U[ i ] * subs_.CTRL_NORMALIZATION[ i ];

	/* hold current controls constant through horizon */
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < NU; ++j) acadoVariables.u[ i * NU + j ] = U[ j ];
	}

	/* store control horizon */
	memcpy(prev_ctrl_horiz_, acadoVariables.u, sizeof(acadoVariables.u));
}

void FwNMPC::updateACADO_X0() {

	double X0[NX];
	paths_.ll2NE(X0[0], X0[1], (double)subs_.glob_pos.latitude, (double)subs_.glob_pos.longitude); // n, e
	X0[2]		= -((double)subs_.glob_pos.altitude - paths_.getHomeAlt()); // d
	X0[3]		= (double)subs_.ekf_ext.airspeed; // V
	X0[4]		= (double)( (subs_.glob_vel.vector.z - subs_.ekf_ext.windZ) / subs_.ekf_ext.airspeed ); // gamma
	X0[5]		= (double)subs_.aslctrl_data.YawAngle; // xi
	X0[6]		= (double)subs_.aslctrl_data.RollAngle; // phi
	X0[7]		= (double)subs_.aslctrl_data.PitchAngle; // theta
	X0[8]		= (double)subs_.aslctrl_data.p; // p
	X0[9]		= (double)subs_.aslctrl_data.q; // q
	X0[10]	= (double)subs_.aslctrl_data.r; // r
	X0[11]	= acadoVariables.x[11]; // throt
	X0[12]	= acadoVariables.x[12]; // xsw

	for (int i = 0; i < NX; ++i) acadoVariables.x0[ i ] = X0[ i ];
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
	OD[23] = (double)subs_.nmpc_params.k_t_d;
	OD[24] = (double)subs_.nmpc_params.e_d_co;
	OD[25] = (double)subs_.nmpc_params.k_t_ne;
	OD[26] = (double)subs_.nmpc_params.e_ne_co;
	OD[27] = (double)subs_.nmpc_params.eps_v;
	OD[28] = (double)subs_.nmpc_params.alpha_p_co;
	OD[29] = (double)subs_.nmpc_params.alpha_m_co;
	OD[30] = (double)subs_.nmpc_params.alpha_delta_co;

	for (int i = 0; i < N+1; ++i) {
			for (int j = 0; j < NOD; ++j) {
					acadoVariables.od[ i * NOD + j ] = OD[ j ];
			}
	}
}

void FwNMPC::updateACADO_Y() {

	/* update references */
	double Y[NY];
	Y[0] 	= 0.0;	// e_t_ne _ref
	Y[1] 	= 0.0;	// e_t_d _ref
	Y[2] 	= 0.0;	// e_v_n _ref
	Y[3] 	= 0.0;	// e_v_e _ref
	Y[4] 	= 0.0; 	// e_v_d _ref
	Y[5] 	= subs_.aslctrl_data.AirspeedRef;	// V_ref
	Y[6] 	= 0.0; 	// p_ref
	Y[7] 	= 0.0;	// q_ref
	Y[8] 	= 0.0;	// r_ref
	Y[7] 	= 0.0;	// alpha_soft _ref
	Y[9] 	= 0.0;	// uthrot _ref
	Y[10] = 0.0;	// phi_ref _ref
	Y[11] = 0.0;	// theta_ref _ref

	for (int i = 0; i < N; ++i) {
	    for (int j = 0; j < NY-NU; ++j) {
	        acadoVariables.y[ i * NY + j ] = Y[ j ];
	    }
	    for (int j = 0; j < NU; ++j) {
	        acadoVariables.y[ i * NY + (NY-NU) + j ] = prev_ctrl_horiz_[ i * NU + j ];
	    }
	}
}

void FwNMPC::updateACADO_W() {

	/* update objective gains */
	double W[NY];
	for (int i = 0; i < NY; i++) W[ i ] = (double)subs_.nmpc_params.Qdiag[ i ];

	// only update diagonal terms
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < NY; ++j) {
//			if ( j<NY-2 ) { // only last 2 have weight decay
				acadoVariables.W[ NY * NY * i + NY * j + j ] = W[ j ];
//			}
//			else { INTEGERDIVISON!!!! DOESNTWORK..
//				acadoVariables.W[ NY * NY * i + NY * j + j ] = W[ j ] * (1.0 - i / N) * (1.0 - i / N);
//			}
		}
	}
	for (int i = 0; i < NYN; ++i) acadoVariables.WN[ i * NYN + i ] =  W[ i ];
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
	ros::ServiceClient wp_pull_client_ = nmpc_.serviceClient<mavros::WaypointPull>("/mavros/mission/pull");
	mavros::WaypointPull wp_pull_srv;

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

void FwNMPC::calculateTrackError(double &e_t_ne, double &e_t_d, const real_t *in_x, const real_t *in_od) {

	const int minus_NU = 0;

	/* optimized intermediate calculations */

	const double t2 = cos(in_x[4]);

//	const double alpha = -in[4]+in[7];
//
//	const double t3 = alpha-in[44]+in[46];
//	const double t4 = 1.0/(in[46]*in[46]);
//	const double t5 = -alpha+in[45]+in[46];

	double Vsafe = in_x[3];
	if (Vsafe<1.0) Vsafe = 1.0;

	const double n_dot = in_od[36-ACADO_NU-ACADO_NX]+Vsafe*t2*cos(in_x[5]);
	const double e_dot = in_od[37-ACADO_NU-ACADO_NX]+Vsafe*t2*sin(in_x[5]);
	const double d_dot = in_od[38-ACADO_NU-ACADO_NX]-Vsafe*sin(in_x[4]);

	/* begin manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

	// CHECK SEGMENT SWITCHING CONDITIONS //TODO: put this in a function!
	const int idx_OD_0 = ACADO_NX+ACADO_NU-minus_NU;
//	bool b_switch_segment = false;
	int pparam_sel = 0;
//	double sw_dot = 0.0;
//	if ( in[12] < 0.05 ) { // check x_sw
//	    const double vel[3] = {n_dot,e_dot,d_dot};
//	    if ( in[idx_OD_0] < 0.5 ) { // path type
//	        b_switch_segment = check_line_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
//	    } else if (in[ACADO_NX+ACADO_NU-minus_NU] < 1.5 ) {
//	        b_switch_segment = check_curve_seg( &in[0], &vel[0], &in[idx_OD_0+1] );
//	    }
//	} else {
//	    b_switch_segment = true;
//	}
//	if (b_switch_segment) {
//	    pparam_sel = 9;
//	    sw_dot = 1.0;
//	}

	double d_n = 0.0;
	double d_e = 0.0;
	double d_d = 0.0;
	double Td_n = 1.0;
	double Td_e = 0.0;
	double Td_d = 0.0;

	const double pparam_type = in_od[pparam_sel];

	// LINE SEGMENT
	if ( pparam_type < 0.5 ) {

	    // variable definitions
	    const double pparam_aa_n = in_od[pparam_sel+1];
	    const double pparam_aa_e = in_od[pparam_sel+2];
	    const double pparam_aa_d = in_od[pparam_sel+3];
	    const double pparam_bb_n = in_od[pparam_sel+4];
	    const double pparam_bb_e = in_od[pparam_sel+5];
	    const double pparam_bb_d = in_od[pparam_sel+6];

	    // calculate vector from waypoint a to b
	    const double abn = pparam_bb_n - pparam_aa_n;
	    const double abe = pparam_bb_e - pparam_aa_e;
	    const double abd = pparam_bb_d - pparam_aa_d;
	    const double norm_ab = sqrt(abn*abn + abe*abe + abd*abd);

	    // calculate tangent
	    if (norm_ab>0.1) {
	        Td_n = abn / norm_ab;
	        Td_e = abe / norm_ab;
	        Td_d = abd / norm_ab;
	    }

	    // dot product
	    const double dot_abunit_ap = Td_n*(in_x[0] - pparam_aa_n) + Td_e*(in_x[1] - pparam_aa_e) + Td_d*(in_x[2] - pparam_aa_d);

	    // point on track
	    d_n = pparam_aa_n + dot_abunit_ap * Td_n;
	    d_e = pparam_aa_e + dot_abunit_ap * Td_e;
	    d_d = pparam_aa_d + dot_abunit_ap * Td_d;

	// CURVE SEGMENT
	} else if ( pparam_type < 1.5 ) {

	    // variable definitions
	    const double pparam_cc_n = in_od[pparam_sel+1];
	    const double pparam_cc_e = in_od[pparam_sel+2];
	    const double pparam_cc_d = in_od[pparam_sel+3];
	    const double pparam_R = in_od[pparam_sel+4];
	    const double pparam_ldir = in_od[pparam_sel+5];
	    const double pparam_gam_sp = in_od[pparam_sel+6];
	    const double pparam_xi0 = in_od[pparam_sel+7];
	    const double pparam_dxi = in_od[pparam_sel+8];

	    // calculate closest point on loiter circle
	    const double cp_n = in_x[0] - pparam_cc_n;
	    const double cp_e = in_x[1] - pparam_cc_e;
	    const double norm_cp = sqrt( cp_n*cp_n + cp_e*cp_e );
	    const double cp_n_unit = (norm_cp>0.1) ? cp_n / norm_cp : 0.0;
	    const double cp_e_unit = (norm_cp>0.1) ? cp_e / norm_cp : 0.0;
	    d_n = pparam_R * cp_n_unit + pparam_cc_n;
	    d_e = pparam_R * cp_e_unit + pparam_cc_e;

	    // calculate tangent
	    Td_n = pparam_ldir * -cp_e_unit;
	    Td_e = pparam_ldir * cp_n_unit;

	    // spiral angular position: [0,2*pi)
	    const double xi_sp = atan2(cp_e_unit, cp_n_unit);
	    double delta_xi = xi_sp-pparam_xi0;

	    // closest point on nearest spiral leg and tangent down component
	    if (pparam_ldir > 0.0 && pparam_xi0 > xi_sp) {

	        delta_xi = delta_xi + 6.28318530718;

	    } else if (pparam_ldir<0.0 && xi_sp>pparam_xi0) {

	        delta_xi = delta_xi - 6.28318530718;

	    }

	    if (fabs(pparam_gam_sp) < 0.001) {

	        d_d = pparam_cc_d;
	        Td_d = 0.0;

	    } else {

	        const double Rtangam = pparam_R * tan(pparam_gam_sp);

	        // spiral height delta for current angle
	        const double delta_d_xi = -delta_xi * pparam_ldir * Rtangam;

	        // end spiral altitude change
	        const double delta_d_sp_end = -pparam_dxi * Rtangam;

	        // nearest spiral leg
	        double delta_d_k = round( (in_x[2] - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;
	        const double delta_d_end_k  = round( (delta_d_sp_end - (pparam_cc_d + delta_d_xi)) / (6.28318530718*Rtangam) ) * 6.28318530718*Rtangam;

	        // check
	        if (delta_d_k * pparam_gam_sp > 0.0) { //NOTE: gam is actually being used for its sign, but writing a sign operator doesnt make a difference here

	            delta_d_k = 0.0;

	        } else if (fabs(delta_d_k) > fabs(delta_d_end_k) ) {

	            delta_d_k = (delta_d_k < 0.0) ? -fabs(delta_d_end_k) : fabs(delta_d_end_k);

	        }

	        // closest point on nearest spiral leg
	        const double delta_d_sp = delta_d_k + delta_d_xi;
	        d_d = pparam_cc_d + delta_d_sp;
	        Td_d = -sin(pparam_gam_sp);
	    }

	    if (fabs(Td_n)<0.01 && fabs(Td_e)<0.01) { // should always have lateral-directional references on curve (this is only when we hit the center of the circle)
	        Td_n=1.0;
	    }
	}

	/* end manual input !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

	const double pd_n = d_n-in_x[0];
	const double pd_e = d_e-in_x[1];
	const double pd_d = d_d-in_x[2];

	e_t_ne = -Td_e*pd_n+Td_n*pd_e;
	e_t_d = pd_d;
}

int FwNMPC::nmpcIteration() {

	int RET[2] = {0, 0};

	/* check mode */ //TODO: should include some checking to make sure not on ground/ other singularity ridden cases //TODO: this does not cover RC loss..
	if (last_ctrl_mode != 5) bModeChanged = true;
	last_ctrl_mode = subs_.aslctrl_data.aslctrl_mode;
	if (subs_.aslctrl_data.aslctrl_mode == 5) {

		/* start timer */
		ros::Time t_start = ros::Time::now();

		int obctrl_status = 0;

		if (bModeChanged) {
			/* first time in loop */

			// update home wp
			// -->initHorizon needs up to date wp info for ned calculation
			const double new_home_wp[3] = {subs_.home_wp.latitude, subs_.home_wp.longitude, subs_.home_wp.altitude};
			paths_.setHomeWp( new_home_wp );
			prev_wp_idx_ = -1;

			// initHorizon BEFORE Y, this is to make sure controls and prev_horiz are reinitialized before reaching Y update.
			initHorizon();
			updateACADO_Y();
			updateACADO_W();

			bModeChanged = false;
		}
		else {
			//regular update

			/* update ACADO states/references/weights */
			updateACADO_X0();
			updateACADO_Y();
			updateACADO_W();
		}

		/* check current waypoint */
		if ( subs_.current_wp.data != prev_wp_idx_ ) {
			// reset switch state, sw, horizon //TODO: encapsulate
			for (int i = 1; i < N + 2; ++i) acadoVariables.x[ i * NX - 1 ] = 0.0;
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

		/* store ctrl horizon before shift */
		memcpy(prev_ctrl_horiz_, acadoVariables.u, sizeof(acadoVariables.u));

		/* take applied control only for previous throttle horizon */
		for (int i = 0; i < N * NU; i+=3)  prev_ctrl_horiz_[ i ] = acadoVariables.u[ NU ];

		/* publish control action */
		uint64_t t_ctrl;
		publishControls(subs_.aslctrl_data.header, t_ctrl, t_start, obctrl_status);

		//TODO: this should be interpolated in the event of Tnmpc ~= Tfcall //TODO: should this be before the feedback step?
		/* Optional: shift the initialization (look in acado_common.h). */
//		shiftStates(1, 0, 0);
//		shiftControls( 0 );

		//TODO: this should be interpolated in the event of Tnmpc ~= Tfcall
		/* shift (or just hold over) augmented states */
		for (int i = NX_AUGM; i < NX; ++i) acadoVariables.x0[i] = acadoVariables.x[i];

		/* publish ACADO variables */ // should this go outside?
		publishAcadoVars();

		/* publish nmpc info */
		publishNmpcInfo(t_start, t_ctrl);

	}
	else {
		// send "alive" status

		/* publish obctrl msg */
		mavros::AslObCtrl obctrl_msg;
		obctrl_msg.timestamp = 0;
		obctrl_msg.uThrot = 0.0f;
		obctrl_msg.uThrot2 = 0.0f; // for monitoring on QGC
		obctrl_msg.uAilR = 0.0f;
		obctrl_msg.uAilL = 0.0f; // for monitoring on QGC
		obctrl_msg.uElev = 0.0f;
		obctrl_msg.obctrl_status = 0;

		obctrl_pub_.publish(obctrl_msg);
	}

	/* return status */
	return (RET[0] != 0 || RET[1] != 0) ? 1 : 0; //perhaps a better reporting method?
}

void FwNMPC::publishControls(std_msgs::Header header, uint64_t &t_ctrl, ros::Time t_start, int obctrl_status) {

	//TODO: this should be interpolated in the event of Tnmpc ~= Tfcall
	/* Apply the NEXT control immediately to the process, first NU components. */

	double ctrl[NU];
	for (int i = 0; i < NU; ++i)  ctrl[ i ] = acadoVariables.u[ NU + i ] / subs_.CTRL_NORMALIZATION[ i ];

	/* saturate controls */
	for (int i = 0; i < NU; ++i) {
		if (ctrl[ i ] < subs_.CTRL_SATURATION[ i ][ 0 ]) ctrl[ i ] = subs_.CTRL_SATURATION[ i ][ 0 ];
		if (ctrl[ i ] > subs_.CTRL_SATURATION[ i ][ 1 ]) ctrl[ i ] = subs_.CTRL_SATURATION[ i ][ 1 ];
	}

	/* re-introduce offset */
	for (int i = 0; i < NU; ++i)  ctrl[ i ] = ctrl[ i ] + subs_.CTRL_OFFSET[ i ];

	/* dead-zone throttle cut */
	if (ctrl[0]<0.16) ctrl[0] = 0.0; //TODO: maybe a better way to do this.. a bit hacky.

	/* solve time in us */
	ros::Duration t_elapsed = ros::Time::now() - t_start;
	uint64_t t_solve_approx = t_elapsed.toNSec()/1000;

	/* calculate track error for monitoring on QGC */
	double e_t_ne=0.0;
	double e_t_d=0.0;
	calculateTrackError(e_t_ne, e_t_d, acadoVariables.x, acadoVariables.od);

	/* publish obctrl msg */
	mavros::AslObCtrl obctrl_msg;
	obctrl_msg.timestamp = t_solve_approx; //this is actually calculated again after some more calculations.. but nice to get an idea on the ground station
	obctrl_msg.uThrot = (float)ctrl[0];
	obctrl_msg.uThrot2 = (float)e_t_d; // for monitoring on QGC
	obctrl_msg.uAilR = (float)ctrl[1];
	obctrl_msg.uAilL = (float)e_t_ne; // for monitoring on QGC
	obctrl_msg.uElev = (float)ctrl[2];
	obctrl_msg.obctrl_status = (uint8_t)obctrl_status;

	obctrl_pub_.publish(obctrl_msg);

	/* update last control timestamp / publish elapsed ctrl loop time */
	t_elapsed = ros::Time::now() - t_lastctrl;
	t_ctrl = t_elapsed.toNSec()/1000;
	t_lastctrl = ros::Time::now();
}

void FwNMPC::publishAcadoVars() {

	fw_ctrl::AcadoVars acado_vars;

	/* measurements */
	for (int i=0; i<NX+1; i++) {
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
	acado_vars.k_t_d = (float)acadoVariables.od[23];
	acado_vars.e_d_co = (float)acadoVariables.od[24];
	acado_vars.k_t_ne = (float)acadoVariables.od[25];
	acado_vars.e_ne_co = (float)acadoVariables.od[26];
	acado_vars.eps_v = (float)acadoVariables.od[27];
	acado_vars.alpha_p_co = (float)acadoVariables.od[28];
	acado_vars.alpha_m_co = (float)acadoVariables.od[29];
	acado_vars.alpha_delta_co = (float)acadoVariables.od[30];

	/* references */ //NOTE: only recording non-zero references
//	acado_vars.y_e_t_ne = (float)acadoVariables.y[0];
//	acado_vars.y_e_t_d = (float)acadoVariables.y[1];
//	acado_vars.y_e_v_n = (float)acadoVariables.y[2];
//	acado_vars.y_e_v_e = (float)acadoVariables.y[3];
//	acado_vars.y_e_v_d = (float)acadoVariables.y[4];
	acado_vars.y_V = (float)acadoVariables.y[5];
//	acado_vars.y_p = (float)acadoVariables.y[6];
//	acado_vars.y_q = (float)acadoVariables.y[7];
//	acado_vars.y_r = (float)acadoVariables.y[8];
//	acado_vars.y_asoft = (float)acadoVariables.y[9];
//	acado_vars.y_uT = (float)acadoVariables.y[10];
//	acado_vars.y_phi_ref = (float)acadoVariables.y[11];
//	acado_vars.y_theta_ref = (float)acadoVariables.y[12];
	acado_vars.y_uT0 = (float)acadoVariables.y[13];
	acado_vars.y_phi_ref0 = (float)acadoVariables.y[14];
	acado_vars.y_theta_ref0 = (float)acadoVariables.y[15];

	acado_vars_pub_.publish(acado_vars);
}

void FwNMPC::publishNmpcInfo(ros::Time t_start, uint64_t t_ctrl) {

	fw_ctrl::NmpcInfo nmpc_info;

	/* solver info */
	nmpc_info.kkt = (double)getKKT();
	nmpc_info.obj = (double)getObjective();

	/* solve time in us */
	ros::Duration t_elapsed = ros::Time::now() - t_start;
	uint64_t t_solve = t_elapsed.toNSec()/1000;
	nmpc_info.t_solve = t_solve;

	/* time elapsed since last control action publication */
	nmpc_info.t_ctrl = t_ctrl;

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
