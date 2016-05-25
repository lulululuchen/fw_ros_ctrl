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
#include <mavros/WaypointPush.h>

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
#define N		ACADO_N		/* Number of intervals in the horizon. */
#define NX_AUGM 3			/* Number of augmented differential state variables. */

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
	void 	globPosCb(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void 	ekfExtCb(const mavros::AslEkfExt::ConstPtr& msg);
	void 	nmpcParamsCb(const mavros::AslNmpcParams::ConstPtr& msg);
	void 	waypointListCb(const mavros::WaypointList::ConstPtr& msg);
	void 	currentWpCb(const std_msgs::Int32::ConstPtr& msg);
	void 	homeWpCb(const geographic_msgs::GeoPoint::ConstPtr& msg);

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
	int 	getLoopRate();

	/* functions */
	void 	update();
	int 	nmpcIteration();
	void 	ll2NE(double &n, double &e, const double lat, const double lon, const double lat0, const double lon0);

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
	ros::Subscriber glob_pos_sub_;
	ros::Subscriber ekf_ext_sub_;
	ros::Subscriber nmpc_params_sub_;
	ros::Subscriber waypoint_list_sub_;
	ros::Subscriber current_wp_sub_;
	ros::Subscriber home_wp_sub_;

	/* publishers */
	// to pixhawk
	ros::Publisher att_sp_pub_;
	// for logging
	ros::Publisher kkt_pub_;
	ros::Publisher obj_pub_;
	ros::Publisher tsolve_pub_;
	ros::Publisher tctrl_pub_;
	// augm states
	ros::Publisher intg_e_t_pub_;
	ros::Publisher intg_e_chi_pub_;
	ros::Publisher sw_pub_;

	/* services */
//	ros::ServiceClient wp_pull_serviceClient_ = nmpc_.serviceClient<mavros::WaypointPull>("/mavros/mission/pull");

	/* time keeping */
	ros::Time	t_lastctrl;

	/* controller switch */
	bool	bModeChanged;
	int		last_ctrl_mode;

	/* control horizon */
	double prev_ctrl_horiz_[ NU * N ];

	/* path definitions */
	int prev_wp_idx_;
	int last_wp_idx_;
	PathManager paths_;

	/* node functions */
	void 	shutdown();

};

} // namespace fw_nmpc

#endif
