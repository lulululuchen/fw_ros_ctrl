#ifndef _SUBS_H
#define _SUBS_H

// INCLUDES for ROS
#include <ros/ros.h>
#include <ros/console.h>

// INCLUDES for mavros
#include <mavros/AslCtrlData.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros/AslEkfExt.h>
#include <mavros/AslNmpcParams.h>

/*
 * class definition for subscription data //TODO: put callbacks here? maybe an init() function for subs? maybe even pubs??
 */

class subscriptions
{

public:

	mavros::AslCtrlData						aslctrl_data;
	geometry_msgs::Vector3Stamped	global_vel;
	geometry_msgs::Vector3Stamped local_pos;
	mavros::AslEkfExt							ekf_ext;
	mavros::AslNmpcParams					nmpc_params;

	/* conversions */
	/* control normalizations / saturations */ //TODO: make this an input (subs)
	const double CTRL_NORMALIZATION[4] = {1.0, 0.349, 0.349, 0.349};
	const double CTRL_SATURATION[4][2] = { {0.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0} };

};

#endif
