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

// INCLUDES for ...
#include "subs.h"
#include "path_manager.h"

// INCLUDES for ACADO
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

/* some convenient definitions */
#define NX	ACADO_NX	/* Number of differential state variables.  */
#define NU 	ACADO_NU	/* Number of control inputs. */
#define NOD	ACADO_NOD	/* Number of online data values. */
#define NY 	ACADO_NY 	/* Number of measurements/references on nodes 0..N - 1. */
#define NYN	ACADO_NYN	/* Number of measurements/references on node N. */
#define N	ACADO_N		/* Number of intervals in the horizon. */
#define NX_AUGM 2		/* Number of augmented differential state variables. */

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
	void 	aslctrlDataCb(const mavros_msgs::AslCtrlData::ConstPtr& msg);
	void 	globPosCb(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void 	globVelCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
	void 	ekfExtCb(const mavros_msgs::AslEkfExt::ConstPtr& msg);
	void 	nmpcParamsCb(const mavros_msgs::AslNmpcParams::ConstPtr& msg);
	void 	waypointListCb(const mavros_msgs::WaypointList::ConstPtr& msg);
	void 	currentWpCb(const std_msgs::Int32::ConstPtr& msg);
	void 	homeWpCb(const mavros_msgs::HomePosition::ConstPtr& msg);
	void 	aslctrlDebugCb(const mavros_msgs::AslCtrlDebug::ConstPtr& msg);

	/* initializations */
	int 	initNMPC();
	void 	initACADOVars();
	void 	initHorizon();

	/* sets */
	void 	updateACADO_X0();
	void	updateACADO_OD();
	void	updateACADO_Y();
	void	updateACADO_W();

	/* gets */
	double	getLoopRate();
	double 	getTimeStep();
	void 	reqSubs();
	void 	calculateTrackError(const real_t *in);

	/* functions */
	void 	update();
	int 	nmpcIteration();

	/* publishing encapsulation */
	void	publishControls(uint64_t &t_ctrl, uint64_t t_iter_approx, int obctrl_status);
	void	publishAcadoVars();
	void	publishNmpcInfo(ros::Time t_iter_start, uint64_t t_ctrl, uint64_t t_solve, uint64_t t_update, uint64_t t_wp_man);

	double	LOOP_RATE;
	double 	TSTEP;
	int 	FAKE_SIGNALS;

private:

	/* subscription data */
	Subscriptions 	subs_;

	/* node handles */
	ros::NodeHandle nmpc_;

	/* subscribers */
	ros::Subscriber aslctrl_data_sub_;
	ros::Subscriber glob_pos_sub_;
	ros::Subscriber glob_vel_sub_;
	ros::Subscriber ekf_ext_sub_;
	ros::Subscriber nmpc_params_sub_;
	ros::Subscriber waypoint_list_sub_;
	ros::Subscriber current_wp_sub_;
	ros::Subscriber home_wp_sub_;
	ros::Subscriber aslctrl_debug_sub_;

	/* publishers */
	ros::Publisher obctrl_pub_;
	ros::Publisher nmpc_info_pub_;
	ros::Publisher acado_vars_pub_;

	/* time keeping */
	ros::Time	t_lastctrl;

	/* controller switch */
	bool	bModeChanged;
	int		last_ctrl_mode;
	int 	obctrl_en_;

	/* path definitions */
	int prev_wp_idx_;
	int last_wp_idx_;
	PathManager paths_;

	/* continuity */
	bool bYawReceived;
	float last_yaw_msg_;

	/* track error */
	float track_error_lat_;
	float track_error_lon_;

	/* weight scalers */
	float W_scale_[NY];

	/* control offsets / saturations / normalizations */ //TODO: this should probably not be hard-coded.
	// uT, phi_ref, theta_ref
	const double CTRL_DEADZONE[3] = {0.2, 0.0, 0.0}; // zero-based deadzone TODO: non-zero-based? asymmetric?
//	const double CTRL_OFFSET[3] = {0.0, 0.0, 0.0}; // constant offset TODO: how to use?
	const double CTRL_NORMALIZATION[3] = {0.8, 1.0, 1.0}; // normlization !!must not be zero!!
	const double CTRL_SATURATION[3][2] = { {0.0, 1.0}, {-0.5236, 0.5236}, {-0.2618, 0.2618} }; // these are saturations for the internal model ATM* TODO: probably should incorporate some saturation for the incoming controls as well

	/* node functions */
	void 	shutdown();

};

} // namespace fw_nmpc

#endif
