#ifndef _FW_SIM_H
#define _FW_SIM_H

// INCLUDES for ROS
#include <ros/ros.h>
#include <ros/console.h>

// INCLUDES for mavros
#include <mavros/ActuatorControl.h>

// INCLUDES for ...
#include <math.h>
#include <subs.h>

namespace fw_sim {
/*
 * @brief fw_sim class
 *
 * class that implements fixed-wing simulation
 */

class States {

public:

	double V;
	double beta;
	double alpha;
	double p;
	double q;
	double r;
	double phi;
	double theta;
	double psi;
	double n;
	double e;
	double d;
	double uThr;
};

class Controls {

public:

	double uThr;
	double uEle;
	double uAil;
	double uRud;
};

class Disturbances {

public:

	double wn;
	double we;
	double wd;
};

class ModelParams {

public:

	// aero/thrust parameters
	double cD0;
	double cDa;
	double cDa2;
	double cL0;
	double cLa;
	double cLa2;
	double cLa3;
	double cLq;
	double cLde;
	double cm0;
	double cma;
	double cmq;
	double cmde;
	double cT0;
	double cT1;
	double cT2;
	double tauT;
	double clb;
	double clp;
	double clr;
	double clda;
	double cYb;
	double cnb;
	double cnp;
	double cnr;
	double cndr;

	// geometry
	double S_wing;
	double b_wing;
	double c_chord;
	double i_thrust;

	// mass/inertia
	double mass;
	double Ixx;
	double Iyy;
	double Izz;
	double Ixz;
};

class NmpcParams {

public:

	double Aw2;
	double Bw2;
	double Cw2;
	double Dw2;
	double alpha_co;
	double Qdiag[15];
};

class FwSim {

public:

	FwSim();

	/* callbacks */
	void 	actuatorCtrlCb(const mavros::ActuatorControl::ConstPtr& msg);

	/* initializations */

	/* functions */
	void 	init();
	void 	update();
	void 	simulationStep();
	void 	publish();

	/* gets */
	int 	getLoopRate();

	double LOOP_RATE;

	/* guidance */
	int 		aslctrl_mode;
	double 	airspeed_ref;
	double 	pitch_angle_ref;
	double 	roll_angle_ref;

	/* constants */
	const double CONST_G = 9.81;
	const double CONST_RHO_AIR = 1.18;

	Subscriptions subs_;

private:

	/* node handles */
	ros::NodeHandle sim_;

	/* subscribers */
	ros::Subscriber act_sp_sub_;

	/* publishers */
	ros::Publisher aslctrl_data_pub_;
	ros::Publisher local_pos_pub_;
	ros::Publisher glob_vel_pub_;
	ros::Publisher ekf_ext_pub_;
	ros::Publisher nmpc_params_pub_;

	/* simulation structs */
	States	 			current_states_;
	Controls 			current_ctrls_;
	Disturbances	current_dstbs_;
	ModelParams		current_model_params_;
	NmpcParams 		current_nmpc_params_;
	double 				current_vel_[3];

	/* node functions */
	void 	shutdown();

};

} // namespace fw_nmpc

#endif
