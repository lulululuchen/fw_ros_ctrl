#ifndef _FW_NMPC_H
#define _FW_NMPC_H

// INCLUDES for ROS
#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

// INCLUDES for mavros
#include <mavros/ActuatorControl.h>

// INCLUDES for ...
#include "subs.h"
#include <math.h>

// INCLUDES for ACADO
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

/* some convenient definitions */
#define NX	ACADO_NX	/* Number of differential state variables.  */
#define NU 	ACADO_NU	/* Number of control inputs. */
#define NOD	ACADO_NOD	/* Number of online data values. */
#define NY 	ACADO_NY 	/* Number of measurements/references on nodes 0..N - 1. */
#define NYN	ACADO_NYN	/* Number of measurements/references on node N. */
#define N		ACADO_N		/* Number of intervals in the horizon. */
#define NX_AUGM 8			/* Number of augmented differential state variables. */

/* global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace fw_nmpc {
/*
 * @brief fw_nmpc class
 *
 * class that implements fixed-wing nmpc
 */
class FwNMPC {

public:

	FwNMPC();

	/* callbacks */
	void 	aslctrlDataCb(const mavros::AslCtrlData::ConstPtr& msg);
	void 	globVelCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
	void 	localPosCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void 	ekfExtCb(const mavros::AslEkfExt::ConstPtr& msg);
	void 	nmpcParamsCb(const mavros::AslNmpcParams::ConstPtr& msg);

	/* initializations */
	int 	initNMPC();
	void 	initACADOVars();
	void 	initHorizon();

	/* sets */
	void 	setACADO_X0(double X0[NX]);
	void 	setACADO_X(double X[NX]);
	void	setACADO_U(double U[NU]);
	void	setACADO_OD(double OD[NOD]);
	void	setACADO_Y(double Y[NY]);
	void	setACADO_W(double W[NY]);

	/* gets/sets */
	int 	getLoopRate();

	/* functions */
	void 	update();
	int 	nmpcIteration();

	/* publishing encapsulation */
	void	publishControls(std_msgs::Header header);
	void	publishAugmStates();
	void	publishNmpcInfo(ros::Time t_start);

	double	LOOP_RATE;

private:

	/* subscription data */
	Subscriptions 	subs_;

	/* node handles */
	ros::NodeHandle nmpc_;

	/* subscribers */
	ros::Subscriber aslctrl_data_sub_;
	ros::Subscriber local_pos_sub_;			// UTM coordinates NE(rel H)
	ros::Subscriber glob_vel_sub_;			// NED
	ros::Subscriber ekf_ext_sub_;
	ros::Subscriber nmpc_params_sub_;

	/* publishers */
	ros::Publisher act_sp_pub_;
	ros::Publisher kkt_pub_;
	ros::Publisher obj_pub_;
	ros::Publisher tsolve_pub_;
	ros::Publisher tctrl_pub_;
	ros::Publisher delT_pub_;
	ros::Publisher intg_e_Va_pub_;
	ros::Publisher intg_e_theta_pub_;
	ros::Publisher intg_e_phi_pub_;
	ros::Publisher x_w2_uT_pub_;
	ros::Publisher x_w2_uE_pub_;
	ros::Publisher x_w2_uA_pub_;
	ros::Publisher x_w2_uR_pub_;

	/* time keeping */
	ros::Time	t_lastctrl;

	/* controller switch */
	bool	bModeChanged;
	int		last_ctrl_mode;

	/* node functions */
	void 	shutdown();

};

} // namespace fw_nmpc

#endif
