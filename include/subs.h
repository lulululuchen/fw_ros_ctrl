#ifndef _SUBS_H
#define _SUBS_H

// INCLUDES for ROS
#include <ros/ros.h>
#include <ros/console.h>

// INCLUDES for mavros
#include <mavros_msgs/AslCtrlData.h>
#include <mavros_msgs/AslEkfExt.h>
#include <mavros_msgs/AslNmpcParams.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/WaypointList.h>
#include <std_msgs/Int32.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/AslCtrlDebug.h>
#include <mavros_msgs/HomePosition.h>

/*
 * class definition for subscription data //TODO: put callbacks here? maybe an init() function for subs? maybe even pubs??
 */

class Subscriptions
{

public:

	mavros_msgs::AslCtrlData 		aslctrl_data;
	sensor_msgs::NavSatFix	 		glob_pos;
	nav_msgs::Odometry 				odom;
	mavros_msgs::AslEkfExt			ekf_ext;
	mavros_msgs::AslNmpcParams		nmpc_params;
	mavros_msgs::WaypointList		waypoint_list;
	std_msgs::Int32					current_wp;
	geographic_msgs::GeoPoint		home_wp;

};

#endif
