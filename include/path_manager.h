#ifndef _PATH_MANAGER_H
#define _PATH_MANAGER_H

// INCLUDES for ROS
#include <ros/ros.h>
#include <ros/console.h>

// INCLUDES for MAVROS
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/Waypoint.h>

// INCLUDES ...
#include <math.h>

// CONSTANTS
#define M_DEG_TO_RAD 0.017453292519943
#define M_RAD_TO_DEG 57.295779513082323
#define M_TWOPI 6.283185307179586
// #define M_PI 3.141592653589793

/*
 * class definition for dubins segment
 */

class DubinsSegment
{						// (line)	// (arc)	// (loiter unlim.)
public:

	int pparam1; 			// type=0 	// type=1	// type=2
	double pparam2; 	// b_n 		// c_n 		// c_n
	double pparam3; 	// b_e		// c_e 		// c_e
	double pparam4;		// b_d		// c_d 		// c_d
	double pparam5; 	// N/A		// +/-R 	// +/-R
	double pparam6; 	// Chi   	// Chi 		// N/A
	double pparam7; 	// Gam   	// Gam 		// N/A

	void setLine( const double exit_bearing, const double line_elevation, double exit_coordinate[3] );
	void setArc( const double signed_radius, const double exit_bearing, const double elevation, double center_coordinate[3] );
	void setLoitUnlim( const double signed_radius, double center_coordinate[3] );
};

/*
 * setLine()
 *
 * sets mavlink waypoint params to dubins line params
 */
void DubinsSegment::setLine( const double exit_bearing, const double elevation, double exit_coordinate[3] ) {

	// bearing and elevation in [rad]
	// exit coordinate in local frame (n,e,d) [m] above home

	pparam1 = 0;
	pparam2 = exit_coordinate[0];
	pparam3 = exit_coordinate[1];
	pparam4 = exit_coordinate[2];
	pparam5 = 0.0;
	pparam6 = exit_bearing;
	pparam7 = elevation;
}

/*
 * setArc()
 *
 * sets mavlink waypoint params to dubins arc params
 */
void DubinsSegment::setArc( const double signed_radius, const double exit_bearing, const double elevation, double center_coordinate[3] ) {

	// bearing and elevation in [rad]
	// signed radius in [m]
	// center coordinate in local frame (n,e,d) [m] above home

	pparam1 = 1;
	pparam2 = center_coordinate[0];
	pparam3 = center_coordinate[1];
	pparam4 = center_coordinate[2];
	pparam5 = signed_radius;
	pparam6 = exit_bearing;
	pparam7 = elevation;
}

void DubinsSegment::setLoitUnlim( const double signed_radius, double center_coordinate[3] ) {

	pparam1 = 2;
	pparam2 = center_coordinate[0];
	pparam3 = center_coordinate[1];
	pparam4 = center_coordinate[2];
	pparam5 = signed_radius;
	pparam6 = 0.0;
	pparam7 = 0.0;
}

/*
 * class definition for path management
 */

class PathManager
{
	double home_wp[3];

public:

	DubinsSegment path_current;
	DubinsSegment path_next;

	void setHomeWp( const double new_home_wp[3] );
	double getHomeAlt() { return home_wp[2]; };
	void copyNext2Current();
	void updatePaths( const mavros_msgs::WaypointList wp_list, int cur_idx );
	void ll2NE( double &n, double &e, const double lat, const double lon);
	void coordinate_from_bearing_and_distance(double lat_start, double lon_start, double bearing, double dist, double *lat_end, double *lon_end);
	double _wrap_2pi(double bearing);
	double _wrap_pi(double bearing);
};

/*
 * setHomeWp()
 *
 * sets the current home waypoint (local origin)
 */
void PathManager::setHomeWp( const double new_home_wp[3] ) {

	home_wp[0] = new_home_wp[0];
	home_wp[1] = new_home_wp[1];
	home_wp[2] = new_home_wp[2];
}

/*
 * copyNext2Current()
 *
 * copies next path definition to current path
 */
void PathManager::copyNext2Current() {

	path_current = path_next;
}

/*
 * updatePaths()
 *
 * manages the current waypoint targets and provides inputs for dubins current and next path construction
 */
void PathManager::updatePaths( const mavros_msgs::WaypointList wp_list, int cur_idx) {

	int next_idx = cur_idx+1;

	// check list size
	const double num_wps = wp_list.waypoints.size();

	double coordinate_ned[3] = {0.0,0.0,-100.0};
	if (cur_idx>=num_wps || cur_idx<0 || num_wps==0) {
		// end mission, set home

		path_next.setLoitUnlim(100.0, coordinate_ned);
		copyNext2Current();
	}
	else if (cur_idx==num_wps-1) {
		// last wp, set home next

		// current path
		switch (wp_list.waypoints[ cur_idx ].command) {
			case 17:
				ll2NE(coordinate_ned[0], coordinate_ned[1], wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long );
				coordinate_ned[2] = -wp_list.waypoints[ cur_idx ].z_alt;
				path_current.setLoitUnlim(wp_list.waypoints[ cur_idx ].param3, coordinate_ned);
				break;

			case 31000:
				ll2NE(coordinate_ned[0], coordinate_ned[1], wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long );
				coordinate_ned[2] = -wp_list.waypoints[ cur_idx ].z_alt;
				path_current.setLine(_wrap_pi(wp_list.waypoints[ cur_idx ].param3 * M_DEG_TO_RAD), wp_list.waypoints[ cur_idx ].param4 * M_DEG_TO_RAD, coordinate_ned);
				break;

			case 31001:
				double lat_center, lon_center;
				coordinate_from_bearing_and_distance(wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long,
					_wrap_pi(wp_list.waypoints[ cur_idx ].param3 * M_DEG_TO_RAD + ((wp_list.waypoints[ cur_idx ].param2 < 0.0) ? -M_PI_2 : M_PI_2)),
					fabs(wp_list.waypoints[ cur_idx ].param2), &lat_center, &lon_center);
				ll2NE(coordinate_ned[0], coordinate_ned[1], lat_center, lon_center);
				coordinate_ned[2] = -wp_list.waypoints[ cur_idx ].z_alt;
				path_current.setArc(wp_list.waypoints[ cur_idx ].param2, _wrap_pi(wp_list.waypoints[ cur_idx ].param3 * M_DEG_TO_RAD), wp_list.waypoints[ cur_idx ].param4 * M_DEG_TO_RAD, coordinate_ned);
				break;

			default:
				path_current.setLoitUnlim(100.0, coordinate_ned);
				break;
		}

		// next path
		path_next.setLoitUnlim(100.0, coordinate_ned);
	}
	else {

		// current path
		switch (wp_list.waypoints[ cur_idx ].command) {
			case 17:
				ll2NE(coordinate_ned[0], coordinate_ned[1], wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long );
				coordinate_ned[2] = -wp_list.waypoints[ cur_idx ].z_alt;
				path_current.setLoitUnlim(wp_list.waypoints[ cur_idx ].param3, coordinate_ned);
				break;

			case 31000:
				ll2NE(coordinate_ned[0], coordinate_ned[1], wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long );
				coordinate_ned[2] = -wp_list.waypoints[ cur_idx ].z_alt;
				path_current.setLine(_wrap_pi(wp_list.waypoints[ cur_idx ].param3 * M_DEG_TO_RAD), wp_list.waypoints[ cur_idx ].param4 * M_DEG_TO_RAD, coordinate_ned);
				break;

			case 31001:
				double lat_center, lon_center;
				coordinate_from_bearing_and_distance(wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long,
					_wrap_pi(wp_list.waypoints[ cur_idx ].param3 * M_DEG_TO_RAD + ((wp_list.waypoints[ cur_idx ].param2 < 0.0) ? -M_PI_2 : M_PI_2)),
					fabs(wp_list.waypoints[ cur_idx ].param2), &lat_center, &lon_center);
				ll2NE(coordinate_ned[0], coordinate_ned[1], lat_center, lon_center);
				coordinate_ned[2] = -wp_list.waypoints[ cur_idx ].z_alt;
				path_current.setArc(wp_list.waypoints[ cur_idx ].param2, _wrap_pi(wp_list.waypoints[ cur_idx ].param3 * M_DEG_TO_RAD), wp_list.waypoints[ cur_idx ].param4 * M_DEG_TO_RAD, coordinate_ned);
				break;

			default:
				path_current.setLoitUnlim(100.0, coordinate_ned);
				break;
		}

		// next path
		switch (wp_list.waypoints[ next_idx ].command) {
			case 17:
				ll2NE(coordinate_ned[0], coordinate_ned[1], wp_list.waypoints[ next_idx ].x_lat, wp_list.waypoints[ next_idx ].y_long );
				coordinate_ned[2] = -wp_list.waypoints[ next_idx ].z_alt;
				path_next.setLoitUnlim(wp_list.waypoints[ next_idx ].param3, coordinate_ned);
				break;

			case 31000:
				ll2NE(coordinate_ned[0], coordinate_ned[1], wp_list.waypoints[ next_idx ].x_lat, wp_list.waypoints[ next_idx ].y_long );
				coordinate_ned[2] = -wp_list.waypoints[ next_idx ].z_alt;
				path_next.setLine(_wrap_pi(wp_list.waypoints[ next_idx ].param3 * M_DEG_TO_RAD), wp_list.waypoints[ next_idx ].param4 * M_DEG_TO_RAD, coordinate_ned);
				break;

			case 31001:
				double lat_center, lon_center;
				coordinate_from_bearing_and_distance(wp_list.waypoints[ next_idx ].x_lat, wp_list.waypoints[ next_idx ].y_long,
					_wrap_pi(wp_list.waypoints[ next_idx ].param3 * M_DEG_TO_RAD + ((wp_list.waypoints[ next_idx ].param2 < 0.0) ? -M_PI_2 : M_PI_2)),
					fabs(wp_list.waypoints[ next_idx ].param2), &lat_center, &lon_center);
				ll2NE(coordinate_ned[0], coordinate_ned[1], lat_center, lon_center);
				coordinate_ned[2] = -wp_list.waypoints[ next_idx ].z_alt;
				path_next.setArc(wp_list.waypoints[ next_idx ].param2, _wrap_pi(wp_list.waypoints[ next_idx ].param3 * M_DEG_TO_RAD), wp_list.waypoints[ next_idx ].param4 * M_DEG_TO_RAD, coordinate_ned);
				break;

			default:
				path_next.setLoitUnlim(100.0, coordinate_ned);
				break;
		}
	}
}

/*
 * ll2NE()
 *
 * convert from lat lon to local north east (small angle approx)
 */
void PathManager::ll2NE(double &n, double &e, const double lat, const double lon) {

	/* From MATLAB lla2flat function:
	 *
	 * Copyright 2010-2011 The MathWorks, Inc.
	 *
	 * References:
	 * [1] Etkin, B., Dynamics of Atmospheric Flight, John Wiley & Sons, New
	 *     York, 1972.
	 * [2] Stevens, B. L., and F. L. Lewis, Aircraft Control and Simulation,
	 *     Second Edition, John Wiley & Sons, New York, 2003.
	 */

	const double f = 1/298.257223563; // WGS-84
	const double R = 	6378137.0; // equatorial radius

	const double rlat0 = home_wp[0]*0.017453292519943;

	const double dlat = (lat-home_wp[0])*0.017453292519943;
	const double dlon = (lon-home_wp[1])*0.017453292519943;

	const double Rn = R/sqrt(1-(2*f-f*f)*sin(rlat0)*sin(rlat0));
	const double Rm = Rn*((1-(2*f-f*f))/(1-(2*f-f*f)*sin(rlat0)*sin(rlat0)));

	n = dlat/atan2(1,Rm);
	e = dlon/atan2(1,Rn*cos(rlat0));
}

void PathManager::coordinate_from_bearing_and_distance(double lat_start, double lon_start, double bearing, double dist, double *lat_end, double *lon_end) {
	/* adapted from Pixhawk geo lib */

	bearing = _wrap_2pi(bearing);
	double radius_ratio = fabs(dist) / 6378137.0;

	const double lat_start_rad = lat_start * M_DEG_TO_RAD;
	const double lon_start_rad = lon_start * M_DEG_TO_RAD;

	*lat_end = asin(sin(lat_start_rad) * cos(radius_ratio) + cos(lat_start_rad) * sin(radius_ratio) * cos(bearing));
	*lon_end = lon_start_rad + atan2(sin(bearing) * sin(radius_ratio) * cos(lat_start_rad), cos(radius_ratio) - sin(lat_start_rad) * sin(*lat_end));

	*lat_end *= M_RAD_TO_DEG;
	*lon_end *= M_RAD_TO_DEG;
}

double PathManager::_wrap_2pi(double bearing) {
	/* adapted from Pixhawk geo lib */

	/* value is inf or NaN */
	if (!std::isfinite(bearing)) {
		return bearing;
	}

	int c = 0;

	while (bearing >= M_TWOPI) {
		bearing -= M_TWOPI;

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;

	while (bearing < 0.0) {
		bearing += M_TWOPI;

		if (c++ > 3) {
			return NAN;
		}
	}

	return bearing;

}

double PathManager::_wrap_pi(double bearing) {
	/* adapted from Pixhawk geo lib */

	/* value is inf or NaN */
	if (!std::isfinite(bearing)) {
		return bearing;
	}

	int c = 0;

	while (bearing >= M_PI) {
		bearing -= M_TWOPI;

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;

	while (bearing < -M_PI) {
		bearing += M_TWOPI;

		if (c++ > 3) {
			return NAN;
		}
	}

	return bearing;
}

#endif
