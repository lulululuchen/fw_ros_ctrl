#include <ros/ros.h>
#include <ros/package.h>

#include <fw_nmpc.h>

using namespace fw_nmpc;

FwNMPC::FwNMPC() :
		LOOP_RATE(20.0), //MAGIC NUMBER
		t_lastctrl({0}),
		bModeChanged(false),
		last_ctrl_mode(0)
{

	ROS_INFO("Instance of NMPC created");

	/* subscribers */
	aslctrl_data_sub_	= nmpc_.subscribe("/mavros/aslctrl/data", 1, &FwNMPC::aslctrlDataCb, this);
	local_pos_sub_ 		= nmpc_.subscribe("/mavros/local", 1, &FwNMPC::localPosCb, this);
	glob_vel_sub_ 		= nmpc_.subscribe("/mavros/gp_vel", 1, &FwNMPC::globVelCb, this);
	ekf_ext_sub_			= nmpc_.subscribe("/mavros/aslekf_extended", 1, &FwNMPC::ekfExtCb, this);
	nmpc_params_sub_ 	= nmpc_.subscribe("/mavros/nmpc_params", 1, &FwNMPC::nmpcParamsCb, this);

	/* publishers */
	act_sp_pub_				= nmpc_.advertise<mavros::ActuatorControl>("/mavros/actuator_control", 10, true);
	kkt_pub_					= nmpc_.advertise<std_msgs::Float32>("/nmpc/info/kkt",10,true);
	obj_pub_					= nmpc_.advertise<std_msgs::Float32>("/nmpc/info/obj",10,true);
	tsolve_pub_				= nmpc_.advertise<std_msgs::UInt64>("/nmpc/info/tsolve",10,true);
	tctrl_pub_				= nmpc_.advertise<std_msgs::UInt64>("/nmpc/info/tctrl",10,true);
	delT_pub_					= nmpc_.advertise<std_msgs::Float32>("/nmpc/augm/delT",10,true);
	intg_e_Va_pub_		= nmpc_.advertise<std_msgs::Float32>("/nmpc/augm/intg_e_Va",10,true);
	intg_e_theta_pub_	= nmpc_.advertise<std_msgs::Float32>("/nmpc/augm/intg_e_theta",10,true);
	intg_e_phi_pub_  	= nmpc_.advertise<std_msgs::Float32>("/nmpc/augm/intg_e_phi",10,true);
	x_w2_uT_pub_ 			= nmpc_.advertise<std_msgs::Float32>("/nmpc/augm/x_w2_uT",10,true);
	x_w2_uE_pub_			= nmpc_.advertise<std_msgs::Float32>("/nmpc/augm/x_w2_uE",10,true);
	x_w2_uA_pub_			= nmpc_.advertise<std_msgs::Float32>("/nmpc/augm/x_w2_uA",10,true);
	x_w2_uR_pub_			= nmpc_.advertise<std_msgs::Float32>("/nmpc/augm/x_w2_uR",10,true);

}

void FwNMPC::aslctrlDataCb(const mavros::AslCtrlData::ConstPtr& msg) {

	subs_.aslctrl_data.header 				= msg->header;
	subs_.aslctrl_data.aslctrl_mode 	= msg->aslctrl_mode;
	subs_.aslctrl_data.PitchAngle 		= msg->PitchAngle;
	subs_.aslctrl_data.q 							= msg->q;
	subs_.aslctrl_data.RollAngle 			= msg->RollAngle;
	subs_.aslctrl_data.p 							= msg->p;
	subs_.aslctrl_data.r 							= msg->r;
	subs_.aslctrl_data.AirspeedRef		= msg->AirspeedRef;
	subs_.aslctrl_data.PitchAngleRef	= msg->PitchAngleRef;
	subs_.aslctrl_data.RollAngleRef		= msg->RollAngleRef;
	subs_.aslctrl_data.uThrot 				= msg->uThrot;
	subs_.aslctrl_data.uElev 					= msg->uElev;
	subs_.aslctrl_data.uAil 					= msg->uAil;
	subs_.aslctrl_data.uRud 					= msg->uRud;

}

void FwNMPC::globVelCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {

	subs_.global_vel.vector.x = msg->vector.x;
	subs_.global_vel.vector.y = msg->vector.y;
	subs_.global_vel.vector.z = msg->vector.z;
}

void FwNMPC::localPosCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

	subs_.local_pos.vector.x = msg->pose.pose.position.x;
	subs_.local_pos.vector.y = msg->pose.pose.position.y;
	subs_.local_pos.vector.z = -msg->pose.pose.position.z; // note conversion from z-altitude to z-down! TODO: this is AMSL.. should use height above ellipsoid. take from ekf ext
}

void FwNMPC::ekfExtCb(const mavros::AslEkfExt::ConstPtr& msg) {

	subs_.ekf_ext.airspeed 			= msg->airspeed;
	subs_.ekf_ext.beta 					= msg->beta;
	subs_.ekf_ext.alpha 				= msg->alpha;
	subs_.ekf_ext.windSpeed 		= msg->windSpeed;
	subs_.ekf_ext.windDirection = msg->windDirection;
}

void FwNMPC::nmpcParamsCb(const mavros::AslNmpcParams::ConstPtr& msg) {

	subs_.nmpc_params.Aw2 			= msg->Aw2;
	subs_.nmpc_params.Bw2 			= msg->Bw2;
	subs_.nmpc_params.Cw2 			= msg->Cw2;
	subs_.nmpc_params.Dw2 			= msg->Dw2;
	subs_.nmpc_params.alpha_co 	= msg->alpha_co;
	subs_.nmpc_params.Qdiag 		= msg->Qdiag;
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
	double X[NX] = {14.0, 0.0, 0.015, 0.0, 0.0, 0.0, 0.0, 0.015, 0.393, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double U[NU] = {0.391, -0.006, 0.0, 0.0};
	double OD[NOD] = {-8.0, 1.0, -127.0, 16.0, 14.0, 0.015, 0.0, 0.0, 0.14};
	double Y[NY] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double W[NY] = {5.0, 50.0, 50.0, 500.0, 0.1, 5.0, 0.1, 0.0, 0.0, 0.0, 100.0, 0.1, 1.0, 0.1, 0.1};

	/* these set a constant value for each variable through the horizon */
	setACADO_X0(X);
	setACADO_X(X);
	setACADO_U(U);
	setACADO_OD(OD);
	setACADO_Y(Y);
	setACADO_W(W);
}

void FwNMPC::setACADO_X0(double X0[NX]) {

	for (int i = 0; i < NX; ++i) acadoVariables.x0[ i ] = X0[ i ];
}

void FwNMPC::setACADO_X(double X[NX]) {

	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NX; ++j) acadoVariables.x[ i * NX + j ] = X[ j ];
	}
}

void FwNMPC::setACADO_U(double U[NU]) {

	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < NU; ++j) acadoVariables.u[ i * NU + j ] = U[ j ];
	}
}

void FwNMPC::setACADO_OD(double OD[NOD]) {

	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NOD; ++j) acadoVariables.od[ i * NOD + j ] = OD[ j ];
	}
}

void FwNMPC::setACADO_Y(double Y[NY]) {

	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NY; ++j) acadoVariables.y[ i * NY + j ] = Y[ j ];
	}
}

void FwNMPC::setACADO_W(double W[NY]) {

	for (int i = 0; i < NY; ++i) {
		for (int j = 0; j < NY; ++j) {
			if ( i==j ) acadoVariables.W[ i * NY + j ] = W[ i ];
			else acadoVariables.W[ i * NY + j ] = 0;
		}
	}

	for (int i = 0; i < NYN; ++i) {
		for (int j = 0; j < NYN; ++j) {
			if ( i==j ) acadoVariables.WN[ i * (NYN) + j ] = W[ i ];
			else acadoVariables.WN[ i * (NYN) + j ] = 0;
		}
	}
}

int FwNMPC::getLoopRate() {
	return LOOP_RATE;
}

void FwNMPC::update() {

	double current_states[NX];
	current_states[0]		= (double)subs_.ekf_ext.airspeed;					// Va
	current_states[1]		= (double)subs_.ekf_ext.beta;							// beta
	current_states[2]		= (double)subs_.ekf_ext.alpha;						// alpha
	current_states[3]		= (double)subs_.aslctrl_data.p;						// p
	current_states[4]		= (double)subs_.aslctrl_data.q;						// q
	current_states[5]		= (double)subs_.aslctrl_data.r;						// r
	current_states[6]		= (double)subs_.aslctrl_data.RollAngle;		// phi
	current_states[7]		= (double)subs_.aslctrl_data.PitchAngle;	// theta
	current_states[8]		= acadoVariables.x0[8];										// delT
	current_states[9]		= acadoVariables.x0[9];										// intg_e_Va
	current_states[10]	= acadoVariables.x0[10];									// intg_e_theta
	current_states[11]	= acadoVariables.x0[11];									// intg_e_phi
	current_states[12]	= acadoVariables.x0[12];									// x_w2_uT
	current_states[13]	= acadoVariables.x0[13];									// x_w2_uE
	current_states[14]	= acadoVariables.x0[14];									// x_w2_uA
	current_states[15]	= acadoVariables.x0[15];									// x_w2_uR
	setACADO_X0(current_states);

	/* update references */
	double current_references[NY];
	current_references[0] 	= 0.0;	// e_Va
	current_references[1] 	= 0.0;	// e_theta
	current_references[2] 	= 0.0;	// e_phi
	current_references[3] 	= 0.0;	// e_beta
	current_references[4] 	= 0.0; 	// p
	current_references[5] 	= 0.0;	// q
	current_references[6] 	= 0.0;	// r
	current_references[7] 	= 0.0;	// intg_e_Va
	current_references[8] 	= 0.0;	// intg_e_theta
	current_references[9] 	= 0.0;	// intg_e_phi
	current_references[11] 	= 0.0;	// Q_alpha
	current_references[12] 	= 0.0;	// z_w2_uT
	current_references[13] 	= 0.0;	// z_w2_uE
	current_references[14] 	= 0.0;	// z_w2_uA
	current_references[15] 	= 0.0;	// z_w2_uR
	setACADO_Y(current_references);

	/* update online data */
	double current_online_data[NOD];
	current_online_data[0] = (double)subs_.nmpc_params.Aw2;							// w2 weight
	current_online_data[1] = (double)subs_.nmpc_params.Bw2;							// w2 weight
	current_online_data[2] = (double)subs_.nmpc_params.Cw2;							// w2 weight
	current_online_data[3] = (double)subs_.nmpc_params.Dw2;							// w2 weight
	current_online_data[4] = (double)subs_.aslctrl_data.AirspeedRef;		// airspeed reference
	current_online_data[5] = (double)subs_.aslctrl_data.PitchAngleRef;	// pitch angle reference
	current_online_data[6] = (double)subs_.aslctrl_data.RollAngleRef;		// roll angle reference
	current_online_data[7] = 0.0;																				// sideslip angle reference
	current_online_data[8] = (double)subs_.nmpc_params.alpha_co;				// angle of attack cut off
	setACADO_OD(current_online_data);

	/* update objective gains */
	double current_objective_gains[NY];
	for (int i = 0; i < NY; i++) current_objective_gains[ i ] = (double)subs_.nmpc_params.Qdiag[ i ];
	setACADO_W(current_objective_gains);
}

void FwNMPC::initHorizon() {

	//TODO: sliding window and/or low pass filter

	/* hold current states constant through horizion */
	for (int i = 0; i < N + 1; ++i) {
		for (int j = 0; j < NX-NX_AUGM; ++j) acadoVariables.x[ i * NX + j ] = acadoVariables.x0[ j ];
	}

	/* initialize augmented states */
	for (int i = 0; i < N + 1; ++i) {
		for (int j = NX_AUGM; j < NX; ++j) acadoVariables.x[ i * NX + j ] = 0.0; //TODO: NOT ALL ZEROS
	}

	/* get current controls */
	double U[NU] = {subs_.aslctrl_data.uThrot, subs_.aslctrl_data.uElev, subs_.aslctrl_data.uAil, subs_.aslctrl_data.uRud}; // w/o CONVERSIONS

	/* saturate controls */
	for (int i = 0; i < NU; ++i) {
		if (U[i] < subs_.CTRL_SATURATION[i][0]) U[i] = subs_.CTRL_SATURATION[i][0];
		if (U[i] > subs_.CTRL_SATURATION[i][1]) U[i] = subs_.CTRL_SATURATION[i][1];
	}

	/* normalize controls */
	for (int i = 0; i < NU; ++i)  U[ i ] = U[ i ] * subs_.CTRL_NORMALIZATION[ i ];

	/* hold current controls constant through horizon */
	setACADO_U(U);
}

int FwNMPC::nmpcIteration() {

	int RET[2] = {0, 0};

	/* check mode */ //TODO: should include some checking to make sure not on ground/ other singularity ridden cases
	bModeChanged = (subs_.aslctrl_data.aslctrl_mode - last_ctrl_mode) > 0 ? false : true;
	last_ctrl_mode = subs_.aslctrl_data.aslctrl_mode;
	if (subs_.aslctrl_data.aslctrl_mode == 7) {

		/* start timer */
		ros::Time t_start = ros::Time::now();

		if (bModeChanged) {
			/* first time in loop */
			initHorizon();
			bModeChanged = false;
		}

		/* Prepare first step */
		RET[0] = preparationStep();

		/* Perform the feedback step. */
		RET[1] = feedbackStep();

		/* publish control action */
		publishControls(subs_.aslctrl_data.header);

		//TODO: this should be interpolated in the event of Tnmpc ~= Tfcall
		/* Optional: shift the initialization (look at acado_common.h). */
		shiftStates(2, 0, 0);
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
	double ctrl[NU];
	for (int i = 0; i < NU; ++i)  ctrl[ i ] = acadoVariables.u[ NU + i ] / subs_.CTRL_NORMALIZATION[ i ];

	/* saturate controls */
	for (int i = 0; i < NU; ++i) {
		if (ctrl[i] < subs_.CTRL_SATURATION[i][0]) ctrl[i] = subs_.CTRL_SATURATION[i][0];
		if (ctrl[i] > subs_.CTRL_SATURATION[i][1]) ctrl[i] = subs_.CTRL_SATURATION[i][1];
	}

	mavros::ActuatorControl actuator_msg;
	actuator_msg.group_mix 		= 1;								// actuator group -- fmu group
	actuator_msg.header 		 	= header;						// return px4 timestamp for latency monitoring --> REMEMBER to change ros::now in actuator_control plugin to take px4 timestamp
	actuator_msg.controls[0] 	= (float)ctrl[2]; 	// CH_AIL_R
	actuator_msg.controls[1] 	= (float)ctrl[1]; 	// CH_ELV
	actuator_msg.controls[2] 	= (float)ctrl[3]; 	// CH_RDR
	actuator_msg.controls[3] 	= (float)ctrl[0]; 	// CH_THR_1
	actuator_msg.controls[4] 	= 0.0f;					 		// CH_FLAPS
	actuator_msg.controls[5] 	= -(float)ctrl[2];	// CH_AIL_L
	act_sp_pub_.publish(actuator_msg);

	/* update last control timestamp / publish elapsed ctrl loop time */
	ros::Duration t_elapsed = ros::Time::now() - t_lastctrl;
	uint64_t tctrl = t_elapsed.toNSec()/1000;
	tctrl_pub_.publish(tctrl);
	t_lastctrl = ros::Time::now();
}

void FwNMPC::publishAugmStates() {

	delT_pub_.publish( (float)acadoVariables.x0[NX-NX_AUGM] );
	intg_e_Va_pub_.publish( (float)acadoVariables.x0[NX-NX_AUGM+1] );
	intg_e_theta_pub_.publish( (float)acadoVariables.x0[NX-NX_AUGM+2] );
	intg_e_phi_pub_.publish( (float)acadoVariables.x0[NX-NX_AUGM+3] );
	x_w2_uT_pub_.publish( (float)acadoVariables.x0[NX-NX_AUGM+4] );
	x_w2_uE_pub_.publish( (float)acadoVariables.x0[NX-NX_AUGM+5] );
	x_w2_uA_pub_.publish( (float)acadoVariables.x0[NX-NX_AUGM+6] );
	x_w2_uR_pub_.publish( (float)acadoVariables.x0[NX-NX_AUGM+7] );
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

	/* initialize states, params, and solver */
	int ret = nmpc.initNMPC();

	if (ret != 0) {
		ROS_ERROR("initNMPC: error in qpOASES QP solver.");
		return 1;
	}

	/* NMPC Loop */
	double loop_rate = nmpc.getLoopRate();
	ros::Rate nmpc_rate(loop_rate);
	while (ros::ok()) {

		/* empty callback queues */
		ros::spinOnce();

		/* update ACADO states/controls */
		nmpc.update();

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
