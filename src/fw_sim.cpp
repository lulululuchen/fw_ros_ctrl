#include <ros/ros.h>
#include <ros/package.h>

#include <fw_sim.h>

using namespace fw_sim;

FwSim::FwSim() :
		LOOP_RATE(100.0), //MAGIC NUMBER
		aslctrl_mode(0),
		airspeed_ref(14.0),
		pitch_angle_ref(0.0),
		roll_angle_ref(0.0),
		current_vel_{0.0, 0.0, 0.0}
{

	ROS_INFO("FW simulation created");

	/* subscribers */
	act_sp_sub_				= sim_.subscribe("/mavros/actuator_control", 1, &FwSim::actuatorCtrlCb, this);

	/* publishers */
	aslctrl_data_pub_	= sim_.advertise<mavros::AslCtrlData>("/mavros/aslctrl/data", 10, true);
	local_pos_pub_ 		= sim_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mavros/local", 10, true);
	glob_vel_pub_ 		= sim_.advertise<geometry_msgs::Vector3Stamped>("/mavros/gp_vel", 10, true);
	ekf_ext_pub_			= sim_.advertise<mavros::AslEkfExt>("/mavros/aslekf_extended", 10, true);
	nmpc_params_pub_ 	= sim_.advertise<mavros::AslNmpcParams>("/mavros/nmpc_params", 10, true);

}

void FwSim::actuatorCtrlCb(const mavros::ActuatorControl::ConstPtr& msg) {

	double controls[4];
	controls[0] = msg->controls[3];
	controls[1] = msg->controls[1];
	controls[2] = msg->controls[0];
	controls[3] = msg->controls[2];

	/* normalize controls */
	for (int i = 0; i < 4; ++i)  controls[ i ] = controls[ i ] * subs_.CTRL_NORMALIZATION[ i ];

	if ( aslctrl_mode == 7 ) {
		current_ctrls_.uThr 	= controls[0];
		current_ctrls_.uEle 	= controls[1];
		current_ctrls_.uAil 	= controls[2];
		current_ctrls_.uRud 	= controls[3];
	}
}

void FwSim::init() {

	/* get model parameters */
	sim_.getParam("sim/model/cD0", current_model_params_.cD0);
	sim_.getParam("sim/model/cDa", current_model_params_.cDa);
	sim_.getParam("sim/model/cDa2", current_model_params_.cDa2);
	sim_.getParam("sim/model/cL0", current_model_params_.cL0);
	sim_.getParam("sim/model/cLa", current_model_params_.cLa);
	sim_.getParam("sim/model/cLa2", current_model_params_.cLa2);
	sim_.getParam("sim/model/cLa3", current_model_params_.cLa3);
	sim_.getParam("sim/model/cLq", current_model_params_.cLq);
	sim_.getParam("sim/model/cLde", current_model_params_.cLde);
	sim_.getParam("sim/model/cm0", current_model_params_.cm0);
	sim_.getParam("sim/model/cma", current_model_params_.cma);
	sim_.getParam("sim/model/cmq", current_model_params_.cmq);
	sim_.getParam("sim/model/cmde", current_model_params_.cmde);
	sim_.getParam("sim/model/cT0", current_model_params_.cT0);
	sim_.getParam("sim/model/cT1", current_model_params_.cT1);
	sim_.getParam("sim/model/cT2", current_model_params_.cT2);
	sim_.getParam("sim/model/tauT", current_model_params_.tauT);
	sim_.getParam("sim/model/clb", current_model_params_.clb);
	sim_.getParam("sim/model/clp", current_model_params_.clp);
	sim_.getParam("sim/model/clr", current_model_params_.clr);
	sim_.getParam("sim/model/clda", current_model_params_.clda);
	sim_.getParam("sim/model/cYb", current_model_params_.cYb);
	sim_.getParam("sim/model/cnb", current_model_params_.cnb);
	sim_.getParam("sim/model/cnp", current_model_params_.cnp);
	sim_.getParam("sim/model/cnr", current_model_params_.cnr);
	sim_.getParam("sim/model/cndr", current_model_params_.cndr);
	sim_.getParam("sim/model/S_wing", current_model_params_.S_wing);
	sim_.getParam("sim/model/b_wing", current_model_params_.b_wing);
	sim_.getParam("sim/model/c_chord", current_model_params_.c_chord);
	sim_.getParam("sim/model/i_thrust", current_model_params_.i_thrust);
	sim_.getParam("sim/model/mass", current_model_params_.mass);
	sim_.getParam("sim/model/Ixx", current_model_params_.Ixx);
	sim_.getParam("sim/model/Iyy", current_model_params_.Iyy);
	sim_.getParam("sim/model/Izz", current_model_params_.Izz);
	sim_.getParam("sim/model/Ixz", current_model_params_.Ixz);

	/* initialize states/ctrls */
	current_states_.V 		= 14.0;
	current_states_.beta 	= 0.0;
	current_states_.alpha = 0.015;
	current_states_.p 		= 0.0;
	current_states_.q 		= 0.0;
	current_states_.r 		= 0.0;
	current_states_.phi 	= 0.0;
	current_states_.theta = 0.015;
	current_states_.psi 	= 0.0;
	current_states_.n 		= 0.0;
	current_states_.e 		= 0.0;
	current_states_.d 		= 0.0;
	current_states_.uThr 	= 0.393;

	current_ctrls_.uThr 	= 0.3913;
	current_ctrls_.uEle 	= -0.0064;
	current_ctrls_.uAil 	= 0.0;
	current_ctrls_.uRud 	= 0.0;
}

void FwSim::update() {

	/* disturbances */
	sim_.getParam("sim/env/wn", current_dstbs_.wn);
	sim_.getParam("sim/env/we", current_dstbs_.we);
	sim_.getParam("sim/env/wd", current_dstbs_.wd);

	/* nmpc parameters */
	sim_.getParam("sim/nmpc/Aw2", current_nmpc_params_.Aw2);
	sim_.getParam("sim/nmpc/Bw2", current_nmpc_params_.Bw2);
	sim_.getParam("sim/nmpc/Cw2", current_nmpc_params_.Cw2);
	sim_.getParam("sim/nmpc/Dw2", current_nmpc_params_.Dw2);
	sim_.getParam("sim/nmpc/alpha_co", current_nmpc_params_.alpha_co);

	std::vector<double> Qdiag_vector;
	sim_.getParam("sim/nmpc/Qdiag", Qdiag_vector);
	for ( unsigned i=0; i < Qdiag_vector.size(); i++ ) {
		current_nmpc_params_.Qdiag[i] = Qdiag_vector[i];
	}

	/* guidance */
	sim_.getParam("sim/guide/aslctrl_mode", aslctrl_mode);
	sim_.getParam("sim/guide/airspeed_ref", airspeed_ref);
	sim_.getParam("sim/guide/pitch_angle_ref", pitch_angle_ref);
	sim_.getParam("sim/guide/roll_angle_ref", roll_angle_ref);
}

void FwSim::simulationStep() {

	States 		states 		= current_states_;
	Controls 	ctrls 		= current_ctrls_;
	States 		d_states;

	/* save some flops */
	double ca 	= cos(states.alpha);
	double sa 	= sin(states.alpha);
	double cph	= cos(states.phi);
	double sph	= sin(states.phi);
	double cth	= cos(states.theta);
	double sth	= sin(states.theta);
	double cps	= cos(states.psi);
	double sps	= sin(states.psi);

	/* control saturations */
	if ( ctrls.uThr > 1.0 ) ctrls.uThr = 1.0;
	if ( ctrls.uThr < 0.0 ) ctrls.uThr = 0.0;
	if ( ctrls.uEle > 0.349 ) ctrls.uEle = 0.349;
	if ( ctrls.uEle < -0.349 ) ctrls.uEle = -0.349;
	if ( ctrls.uEle > 0.349 ) ctrls.uEle = 0.349;
	if ( ctrls.uEle < -0.349 ) ctrls.uEle = -0.349;
	if ( ctrls.uEle > 0.349 ) ctrls.uEle = 0.349;
	if ( ctrls.uEle < -0.349 ) ctrls.uEle = -0.349;

	/* -- model construction -- */

	/* non-dimensionalization */
	if ( states.V < 0.1 ) states.V = 0.1;
	double p_hat		= states.p * current_model_params_.b_wing / (2.0 * states.V);
	double q_hat		= states.q * current_model_params_.c_chord / (2.0 * states.V);
	double r_hat		= states.r * current_model_params_.b_wing / (2.0 * states.V);
	double q_bar_S 	= 0.5 * CONST_RHO_AIR * pow(states.V, 2.0) * current_model_params_.S_wing;

	/* stability axis aerodynamic force coefficients */
	double cD				= current_model_params_.cD0 + current_model_params_.cDa * states.alpha + current_model_params_.cDa2 * pow(states.alpha, 2.0);
	double cY_s    	= current_model_params_.cYb * states.beta;
	double cL_poly 	= current_model_params_.cL0 + current_model_params_.cLa * states.alpha + current_model_params_.cLa2 * pow(states.alpha, 2.0) + current_model_params_.cLa3 * pow(states.alpha, 3.0);
	double cL_stall	= -1.257 * states.alpha + 1.248;
	double lgstc   	= 1.0 / (1.0 + exp(-100.0 * (states.alpha - 0.257)));
	double cL      	= (1.0 - lgstc) * cL_poly + lgstc * cL_stall;

	/* body frame aerodynamic forces */
	double X = q_bar_S * ( ca * -cD - sa * -cL );
	double Y = q_bar_S * cY_s;
	double Z = q_bar_S * ( sa * -cD + ca * -cL );

	/* stability axis aerodynamic moment coefficients */
	double cl_s	= current_model_params_.clb * states.beta + current_model_params_.clp * p_hat + current_model_params_.clr*r_hat + current_model_params_.clda * ctrls.uAil;
	double cm_s	= current_model_params_.cm0 + current_model_params_.cma * states.alpha + current_model_params_.cmq * q_hat + current_model_params_.cmde * ctrls.uEle;
	double cn_s	= current_model_params_.cnb * states.beta + current_model_params_.cnp * p_hat + current_model_params_.cnr*r_hat + current_model_params_.cndr * ctrls.uRud;

	/* body frame aerodynamic moments */
	double Lm = q_bar_S * current_model_params_.b_wing * cl_s;
	double Mm = q_bar_S * current_model_params_.c_chord * cm_s;
	double Nm = q_bar_S * current_model_params_.b_wing * cn_s;

	/* thrust force */
	double T = current_model_params_.cT0 + current_model_params_.cT1 * states.uThr + current_model_params_.cT2 * pow(states.uThr, 2.0);                                                              // neglecting 2nd order term

	/* intermediate states */
	double u = states.V * ca * cos(states.beta);
	double v = states.V * sin(states.beta);
	double w = states.V * sa * cos(states.beta);

	/* intermediate state differentials */
	double u_dot = states.r * v - states.q * w - CONST_G * sth + (X + T * cos(current_model_params_.i_thrust)) / current_model_params_.mass;
	double v_dot = states.p * w - states.r * u + CONST_G * sph * cth + Y / current_model_params_.mass;
	double w_dot = states.q * u - states.p * v + CONST_G * cph * cth + (Z + T * sin(current_model_params_.i_thrust)) / current_model_params_.mass;

	/* state differentials */
	d_states.V			= (u * u_dot + v * v_dot + w * w_dot) / states.V;
	d_states.beta   = (states.V * v_dot - v * d_states.V) / (pow(states.V, 2.0) * cos(states.beta));
	d_states.alpha  = (u * w_dot - w * u_dot) / (pow(u, 2.0) + pow(w, 2.0));

	double I1   = current_model_params_.Ixz * (current_model_params_.Iyy - current_model_params_.Ixx - current_model_params_.Izz);
	double I2   = (current_model_params_.Ixx * current_model_params_.Izz - pow(current_model_params_.Ixz, 2.0));
	d_states.p	= (current_model_params_.Izz * Lm + current_model_params_.Ixz * Nm - (I1 * states.p + (pow(current_model_params_.Ixz, 2.0) + current_model_params_.Izz * (current_model_params_.Izz - current_model_params_.Iyy)) * states.r) * states.q) / I2;
	d_states.q  = (Mm - (current_model_params_.Ixx - current_model_params_.Izz) * states.p * states.r - current_model_params_.Ixz * (pow(states.p, 2.0) - pow(states.r, 2.0))) / current_model_params_.Iyy;
	d_states.r  = (current_model_params_.Ixz * Lm + current_model_params_.Ixx * Nm + (I1 * states.r + (pow(current_model_params_.Ixz, 2.0) + current_model_params_.Ixx * (current_model_params_.Ixx - current_model_params_.Iyy)) * states.p) * states.q) / I2;

	d_states.phi		= states.p + (states.q * sph + states.r * cph) * tan(states.theta);
	d_states.theta  = states.q * cph - states.r * sph;
	d_states.psi    = (states.q * sph + states.r * cph) / cth;

	d_states.n = u * cth * cps + v * (sph * sth * cps - cph * sps) + w * (sph * sps + cph * sth * cps) + current_dstbs_.wn;
	d_states.e = u * cth * sps + v * (cph * cps + sph * sth * sps) + w * (cph * sth * sps - sph * cps) + current_dstbs_.we;
	d_states.d = -u * sth + v * sph * cth + w * cph * cth + current_dstbs_.wd;

	d_states.uThr = (ctrls.uThr - states.uThr) / current_model_params_.tauT;

	/* integrate and update current system states */
	current_states_.V 		= states.V + 1.0 / LOOP_RATE * d_states.V;
	current_states_.beta 	= states.beta + 1.0 / LOOP_RATE * d_states.beta;
	current_states_.alpha = states.alpha + 1.0 / LOOP_RATE * d_states.alpha;
	current_states_.p 		= states.p + 1.0 / LOOP_RATE * d_states.p;
	current_states_.q 		= states.q + 1.0 / LOOP_RATE * d_states.q;
	current_states_.r 		= states.r + 1.0 / LOOP_RATE * d_states.r;
	current_states_.phi 	= states.phi + 1.0 / LOOP_RATE * d_states.phi;
	current_states_.theta = states.theta + 1.0 / LOOP_RATE * d_states.theta;
	current_states_.psi 	= states.psi + 1.0 / LOOP_RATE * d_states.psi;
	current_states_.n 		= states.n + 1.0 / LOOP_RATE * d_states.n;
	current_states_.e 		= states.e + 1.0 / LOOP_RATE * d_states.e;
	current_states_.d 		= states.d + 1.0 / LOOP_RATE * d_states.d;
	current_states_.uThr 	= states.uThr + 1.0 / LOOP_RATE * d_states.uThr;

	/* update current vel */
	current_vel_[0] = d_states.n;
	current_vel_[1] = d_states.e;
	current_vel_[2] = d_states.d;

}

void FwSim::publish() {

	/* publish aslctrl data */
	mavros::AslCtrlData aslctrl_data_msg;

	aslctrl_data_msg.aslctrl_mode 	= aslctrl_mode;
	aslctrl_data_msg.PitchAngle 		= current_states_.theta;
	aslctrl_data_msg.q 							= current_states_.q;
	aslctrl_data_msg.RollAngle 			= current_states_.phi;
	aslctrl_data_msg.p 							= current_states_.p;
	aslctrl_data_msg.r 							= current_states_.r;
	aslctrl_data_msg.AirspeedRef		= airspeed_ref; //TODO: set loiter command
	aslctrl_data_msg.PitchAngleRef	= pitch_angle_ref;
	aslctrl_data_msg.RollAngleRef		= roll_angle_ref;
	aslctrl_data_msg.uThrot 				= current_ctrls_.uThr;
	aslctrl_data_msg.uElev 					= current_ctrls_.uEle;
	aslctrl_data_msg.uAil 					= current_ctrls_.uAil;
	aslctrl_data_msg.uRud 					= current_ctrls_.uRud;

	aslctrl_data_pub_.publish(aslctrl_data_msg);

	/* publish local position */
	geometry_msgs::PoseWithCovarianceStamped local_pos_msg;

	local_pos_msg.pose.pose.position.x = current_states_.e;
	local_pos_msg.pose.pose.position.y = current_states_.n;
	local_pos_msg.pose.pose.position.z = -current_states_.d;

	local_pos_pub_.publish(local_pos_msg);

	/* publish global velocity */
	geometry_msgs::Vector3Stamped glob_vel_msg;

	glob_vel_msg.vector.x = current_vel_[1];
	glob_vel_msg.vector.y = current_vel_[0];
	glob_vel_msg.vector.z = current_vel_[2];

	glob_vel_pub_.publish(glob_vel_msg);

	/* publish ekf extended */
	mavros::AslEkfExt ekf_ext_msg;

	ekf_ext_msg.airspeed 				= current_states_.V;
	ekf_ext_msg.beta 						= current_states_.beta;
	ekf_ext_msg.alpha 					= current_states_.alpha;
	ekf_ext_msg.windSpeed				= sqrt(current_dstbs_.wn * current_dstbs_.wn + current_dstbs_.we * current_dstbs_.we + current_dstbs_.wd * current_dstbs_.wd);
	ekf_ext_msg.windDirection		= ekf_ext_msg.windSpeed > 0 ? atan2(current_dstbs_.wn, current_dstbs_.we) : 0.0;
	ekf_ext_msg.windZ						= current_dstbs_.wd;

	ekf_ext_pub_.publish(ekf_ext_msg);

	/* publish nmpc parameters */
	mavros::AslNmpcParams nmpc_params_msg;

	nmpc_params_msg.Aw2 			= current_nmpc_params_.Aw2;
	nmpc_params_msg.Bw2 			= current_nmpc_params_.Bw2;
	nmpc_params_msg.Cw2 			= current_nmpc_params_.Cw2;
	nmpc_params_msg.Dw2 			= current_nmpc_params_.Dw2;
	nmpc_params_msg.alpha_co 	= current_nmpc_params_.alpha_co;
	for (int i = 0; i < 15; ++i) { //TODO: MAGIC NUMBER.. get size of array?
		nmpc_params_msg.Qdiag[ i ] = current_nmpc_params_.Qdiag[ i ];
	}

	nmpc_params_pub_.publish(nmpc_params_msg);
}

int FwSim::getLoopRate() {
	return LOOP_RATE;
}

void FwSim::shutdown() {
	ROS_INFO("Shutting down fixed-wing simulation...");
	ros::shutdown();
}

int main(int argc, char **argv)
{

	/* initialize node */
	ros::init(argc, argv, "fw_sim");
	fw_sim::FwSim sim;

	/* initialize simulation states and params */
	sim.init();

	/* simulation loop */
	int loop_rate = sim.getLoopRate();
	ros::Rate sim_rate(loop_rate);
	while (ros::ok()) {

		/* empty callback queues */
		ros::spinOnce();

		/* update params/vars */
		sim.update();

		/* propagate aircraft model */
		sim.simulationStep();

		/* publish new states */
		sim.publish();

		/* sleep */
		sim_rate.sleep();
	}

	ROS_INFO("fw_sim: closing...");

	return 0;
}
