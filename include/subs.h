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
#include <geometry_msgs/Vector3Stamped.h>

/*
 * class definition for subscription data //TODO: put callbacks here? maybe an init() function for subs? maybe even pubs??
 */

class Subscriptions
{

public:

	mavros::AslCtrlData						aslctrl_data;
	sensor_msgs::NavSatFix	 			glob_pos;
	geometry_msgs::Vector3Stamped glob_vel;
	mavros::AslEkfExt							ekf_ext;
	mavros::AslNmpcParams					nmpc_params;
	mavros::WaypointList					waypoint_list;
	std_msgs::Int32								current_wp;
	geographic_msgs::GeoPoint			home_wp;

	/* control offsets / saturations / normalizations */ //TODO: make this an input (subs), i.e. not hard-coded.
	// uT, phi_ref, theta_ref
	const double CTRL_OFFSET[3] = {0.15, 0.0, 0.0}; // offsets (e.g. dead-zone in throttle)
	const double CTRL_SATURATION[3][2] = { {0.0, 0.85}, {-0.5236, 0.5236}, {-0.2618, 0.2618} }; // these are saturations POST offset removal.
	const double CTRL_NORMALIZATION[3] = {1.0, 1.0, 1.0}; // scale factor for solver controls

};

#endif
