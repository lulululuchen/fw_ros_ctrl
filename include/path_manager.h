#ifndef _PATH_MANAGER_H
#define _PATH_MANAGER_H

// INCLUDES for ROS
#include <ros/ros.h>
#include <ros/console.h>

// INCLUDES for MAVROS
#include <mavros/WaypointList.h>
#include <mavros/Waypoint.h>

// INCLUDES ...
#include <math.h>

/*
 * class definition for dubins segment
 */

class DubinsSegment
{									// (line)		// (curve)
public:

	int pparam1; 		// type=0 	// type=1
	double pparam2; // a_n 			// c_n
	double pparam3; // a_e			// c_e
	double pparam4;	// a_d			// c_d
	double pparam5; // b_n			// R
	double pparam6; // bb_e     // dir
	double pparam7; // bb_d     // gam
	double pparam8; // --       // xi0
	double pparam9; // --       // dxi

	void setLine( const double wp_a[2], const double wp_b[2] );
	void setCurve( const double wp_a[2], const double wp_b[2], const double wp_c[2], double R );
};

/*
 * setLine()
 *
 * translates waypoints to dubins line
 */
void DubinsSegment::setLine( const double wp_a[2], const double wp_b[2] ) {

	pparam1 = 0;
	pparam2 = wp_a[0];
	pparam3 = wp_a[1];
	pparam4 = 0.0;
	pparam5 = wp_b[0];
	pparam6 = wp_b[1];
	pparam7 = 0.0;
	pparam8 = 0.0;
	pparam9 = 0.0;
}

/*
 * setCurve()
 *
 * translates waypoints to dubins curve
 */
void DubinsSegment::setCurve( const double wp_a[2], const double wp_b[2], const double wp_c[2], double R ) {

	pparam1 = 1;
	pparam2 = wp_c[0];
	pparam3 = wp_c[1];
	pparam4 = 0.0;
	pparam5 = fabs(R);
	if ( fabs(R) < 0.01 ) R = 0.01;
	pparam6 = ( R < 0.0 ) ? -1.0 : 1.0;
	pparam7 = 0.0;
	pparam8 = atan2(wp_a[1]-wp_c[1],wp_a[0]-wp_c[0]);
	const double xiend = atan2(wp_b[1]-wp_c[1],wp_b[0]-wp_c[0]);
  double delta_xi = xiend-pparam8;
  if (pparam6 > 0.0 && pparam8 > xiend) {
      delta_xi = delta_xi + 6.28318530718;
  } else if (pparam6 < 0.0 && xiend > pparam8) {
      delta_xi = delta_xi - 6.28318530718;
  }
	pparam9 = delta_xi;
}

/*
 * class definition for path management
 */

class PathManager
{
	double home_wp[2];

public:

	DubinsSegment path_current;
	DubinsSegment path_next;

	void setHomeWp( const double new_home_wp[2] );
	void copyNext2Current();
	void updatePaths( const mavros::WaypointList wp_list, int cur_idx, int prev_idx );
	void ll2NE( double &n, double &e, const double lat, const double lon);
};

/*
 * setHomeWp()
 *
 * sets the current home waypoint (local origin)
 */
void PathManager::setHomeWp( const double new_home_wp[2] ) {

	home_wp[0] = new_home_wp[0];
	home_wp[1] = new_home_wp[1];
}

/*
 * copyNext2current()
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
void PathManager::updatePaths( const mavros::WaypointList wp_list, int cur_idx, int prev_idx ) {

	double wp_a[2];
	double wp_b[2];
	double wp_c[2];

	int next_idx = cur_idx+1;

	// check list size
	const double num_wps = wp_list.waypoints.size();

	// if only one wp
	if ( num_wps == 1 ) {

		cur_idx = 0;
		if ( wp_list.waypoints[ cur_idx ].command == 17 ) {

			ll2NE(wp_c[0], wp_c[1], wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long );
			wp_a[0] = wp_c[0] + (double)wp_list.waypoints[ cur_idx ].param3;
			wp_a[1] = wp_c[1];

			path_next.setCurve( wp_a, wp_a, wp_c, (double)wp_list.waypoints[ cur_idx ].param3 );
			copyNext2Current();

			return;

		} else {

			// set home loiter
			wp_c[0] = 0.0;
			wp_c[1] = 0.0;
			wp_a[0] = 100.0; // TODO: MAGIC NUMBER
			wp_a[1] = 0.0;

			path_next.setCurve( wp_a, wp_a, wp_c, 100.0 );
			copyNext2Current();

			return;

		}

	} else if ( num_wps > 1 ) { // NOTE: there are quite some corner cases inbetween these... but just dont set bullshit during the experiments.

		// are we on first waypoint? (check 1) // NOTE: only considering loiter case with attached next NAV wp, or NAV wp case with subsequent NAV wp.
		if ( prev_idx < 0 ) {

			if ( cur_idx == 0 ) {

				if ( wp_list.waypoints[ cur_idx ].command == 16 ) {

					prev_idx = cur_idx;
					cur_idx = cur_idx + 1; // AGAIN: SHOULD be subsequent NAV wp

				} else if ( wp_list.waypoints[ cur_idx ].command == 17 ) {

					prev_idx = cur_idx+1; // set arbitrary point for loiter start
				}
			}
		}
		// check for previous DO JUMP
		else if ( wp_list.waypoints[ prev_idx ].command == 177 ) {

			prev_idx = prev_idx - 1;
		}
		// check for current DO JUMP -- this is more of a corner case for if the do jump streams before catching and switching
		else if ( wp_list.waypoints[ cur_idx ].command == 177 ) {

			// switch to target -- NOTE: would be better to check if repeats are complete, but we do not have this information. this will cause some improper planning in the horizon.
			cur_idx = (int)wp_list.waypoints[ cur_idx ].param1;
		}

		// check for terminal index
		if ( cur_idx >= num_wps - 1 ) { // NOTE: DONT PUT DO JUMP LAST, AND ALWAYS PUT LOITER LAST (unless add own accept radii functionality here)

			// check for greater than terminal index (could happen from do jump logic above .. i.e. if someone puts a do jump last.. )
			if ( cur_idx > num_wps - 1 ) {

				// return home
				wp_c[0] = 0.0;
				wp_c[1] = 0.0;
				wp_a[0] = 100.0; // TODO: MAGIC NUMBER
				wp_a[1] = 0.0;

				path_next.setCurve( wp_a, wp_a, wp_c, 100.0 );
				copyNext2Current();

				return;
			}

			// if loiter point, use given loiter radius
			if ( wp_list.waypoints[ cur_idx ].command == 17 ) {

				ll2NE(wp_c[0], wp_c[1], wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long );
				wp_a[0] = wp_c[0] + 100.0;
				wp_a[1] = wp_c[1];

				path_next.setCurve( wp_a, wp_a, wp_c, (double)wp_list.waypoints[ cur_idx ].param3 );
				copyNext2Current();
			}
			// else force set end loiter
			else {

				ll2NE(wp_c[0], wp_c[1], wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long );
				wp_a[0] = wp_c[0] + 100.0;
				wp_a[1] = wp_c[1];

				path_next.setCurve( wp_a, wp_a, wp_c, 100.0 );
				copyNext2Current();
			}

			return;
		}
		// all other cases
		else {

			// curve segment
			if ( wp_list.waypoints[ cur_idx ].command == 17 ||
					( wp_list.waypoints[ cur_idx ].command == 16 && wp_list.waypoints[ cur_idx+1 ].command == 17 && wp_list.waypoints[ prev_idx ].command == 17 ) ) { // catches corner case

				if ( wp_list.waypoints[ cur_idx ].command == 16 && wp_list.waypoints[ cur_idx+1 ].command == 17 && wp_list.waypoints[ prev_idx ].command == 17 ) {
					prev_idx = cur_idx;
					cur_idx = cur_idx + 1;
				}

				ll2NE(wp_a[0], wp_a[1], wp_list.waypoints[ prev_idx ].x_lat, wp_list.waypoints[ prev_idx ].y_long );
				ll2NE(wp_b[0], wp_b[1], wp_list.waypoints[ cur_idx+1 ].x_lat, wp_list.waypoints[ cur_idx+1 ].y_long ); // NOTE: assumes there must be a NAV wp before and after every loiter point
				ll2NE(wp_c[0], wp_c[1], wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long );

				path_current.setCurve( wp_a, wp_b, wp_c, wp_list.waypoints[ cur_idx ].param3 );

				// next is do jump
				if ( wp_list.waypoints[ cur_idx+1 ].command == 177 ) {

					next_idx = wp_list.waypoints[ cur_idx+1 ].param1;

				} else {

					next_idx = cur_idx+1;
				}

				// next is line
				if ( wp_list.waypoints[ next_idx+1 ].command == 16 ) {

					ll2NE(wp_a[0], wp_a[1], wp_list.waypoints[ next_idx ].x_lat, wp_list.waypoints[ next_idx ].y_long );
					ll2NE(wp_b[0], wp_b[1], wp_list.waypoints[ next_idx+1 ].x_lat, wp_list.waypoints[ next_idx+1 ].y_long );

					path_next.setLine( wp_a, wp_b );
				}
				// next is curve
				else if ( wp_list.waypoints[ next_idx+1 ].command == 17 ) {

					ll2NE(wp_c[0], wp_c[1], wp_list.waypoints[ next_idx+1 ].x_lat, wp_list.waypoints[ next_idx+1 ].y_long );
					wp_a[0] = wp_c[0] + wp_list.waypoints[ next_idx+1 ].param3;
					wp_a[1] = wp_c[1];

					path_next.setCurve( wp_a, wp_a, wp_c, wp_list.waypoints[ next_idx+1 ].param3 );
				}
				// return home
				else {

					// set home loiter
					wp_c[0] = 0.0;
					wp_c[1] = 0.0;
					wp_a[0] = 100.0; // TODO: MAGIC NUMBER
					wp_a[1] = 0.0;

					path_next.setCurve( wp_a, wp_a, wp_c, 100.0 );
				}

			}
			// line segment
			else if ( ( wp_list.waypoints[ cur_idx ].command == 16 &&  wp_list.waypoints[ prev_idx ].command == 16 ) ||
					( wp_list.waypoints[ cur_idx ].command == 16 &&  wp_list.waypoints[ prev_idx ].command == 17 && wp_list.waypoints[ cur_idx+1 ].command == 16 ) ) {

				if ( wp_list.waypoints[ cur_idx ].command == 16 &&  wp_list.waypoints[ prev_idx ].command == 17 && wp_list.waypoints[ cur_idx+1 ].command == 16 ) {
					prev_idx = cur_idx;
					cur_idx = cur_idx + 1;
				}

				ll2NE(wp_a[0], wp_a[1], wp_list.waypoints[ prev_idx ].x_lat, wp_list.waypoints[ prev_idx ].y_long );
				ll2NE(wp_b[0], wp_b[1], wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long );

				path_current.setLine( wp_a, wp_b );

				// next is do jump
				if ( wp_list.waypoints[ cur_idx+1 ].command == 177 ) {

					next_idx = wp_list.waypoints[ cur_idx+1 ].param1;

				} else {

					next_idx = cur_idx+1;
				}

				// next is line
				if ( wp_list.waypoints[ next_idx ].command == 16 ) {

					ll2NE(wp_a[0], wp_a[1], wp_list.waypoints[ cur_idx ].x_lat, wp_list.waypoints[ cur_idx ].y_long );
					ll2NE(wp_b[0], wp_b[1], wp_list.waypoints[ next_idx ].x_lat, wp_list.waypoints[ next_idx ].y_long );

					path_next.setLine( wp_a, wp_b );
				}
				// next is curve
				else if ( wp_list.waypoints[ next_idx ].command == 17 ) {

					ll2NE(wp_c[0], wp_c[1], wp_list.waypoints[ next_idx ].x_lat, wp_list.waypoints[ next_idx ].y_long );
					wp_a[0] = wp_c[0] + wp_list.waypoints[ next_idx ].param3;
					wp_a[1] = wp_c[1];

					path_next.setCurve( wp_a, wp_a, wp_c, wp_list.waypoints[ next_idx+1 ].param3 );
				}
				// return home
				else {

					// set home loiter
					wp_c[0] = 0.0;
					wp_c[1] = 0.0;
					wp_a[0] = 100.0; // TODO: MAGIC NUMBER
					wp_a[1] = 0.0;

					path_next.setCurve( wp_a, wp_a, wp_c, 100.0 );
				}

			}
			// else
			else {

				// return home
				wp_c[0] = 0.0;
				wp_c[1] = 0.0;
				wp_a[0] = 100.0; // TODO: MAGIC NUMBER
				wp_a[1] = 0.0;

				path_next.setCurve( wp_a, wp_a, wp_c, 100.0 );
				copyNext2Current();
			}

			return;
		}

	} else {

		// set home loiter
		wp_c[0] = 0.0;
		wp_c[1] = 0.0;
		wp_a[0] = 100.0; // TODO: MAGIC NUMBER
		wp_a[1] = 0.0;

		path_next.setCurve( wp_a, wp_a, wp_c, 100.0 );
		copyNext2Current();

		return;
	}
}

/*
 * ll2NE()
 *
 * convert from lat lon to local north east (small angle approx)
 */
void PathManager::ll2NE(double &n, double &e, const double lat, const double lon) {

	const double ff = 0.006694397995866; // ff = 2*f-f^2 , f = 1/298.25642
	const double R = 6378136.6;

	const double dlat = (lat-home_wp[0])*0.017453292519943;
	const double dlon = (lon-home_wp[1])*0.017453292519943;

	const double sinlat0 = sin(home_wp[0]*0.017453292519943);
	const double ffsin2lat0 = ff*sinlat0*sinlat0;
	const double RN = R / sqrt(1-ffsin2lat0);
	const double RM = RN * (1-ff) / (1-ffsin2lat0);

	n = dlat/atan(1 / RM);
	e = dlon/atan(1 / ( RN * cos(lat*0.017453292519943) ));
}

#endif
