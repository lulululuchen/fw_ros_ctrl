#ifndef _SUBS_H
#define _SUBS_H

// INCLUDES for ROS
#include <ros/ros.h>
#include <ros/console.h>

// INCLUDES for mavros
#include <mavros/AslCtrlData.h>
#include <mavros/AslEkfExt.h>
#include <mavros/AslNmpcParams.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros/WaypointList.h>
#include <std_msgs/Int32.h>
#include <geographic_msgs/GeoPoint.h>

/*
 * class definition for subscription data //TODO: put callbacks here? maybe an init() function for subs? maybe even pubs??
 */

class Subscriptions
{

public:

	mavros::AslCtrlData						aslctrl_data;
	sensor_msgs::NavSatFix	 			glob_pos;
	mavros::AslEkfExt							ekf_ext;
	mavros::AslNmpcParams					nmpc_params;
	mavros::WaypointList					waypoint_list;
	std_msgs::Int32								current_wp;
	geographic_msgs::GeoPoint			home_wp;

	/* control normalizations / saturations */ //TODO: make this an input (subs)
	const double CTRL_NORMALIZATION = 1.0;
	const double CTRL_SATURATION[2] = {-0.6, 0.6};

};

#endif
