#include <fw_nmpc/common/helpers.h>
#include <math.h>

namespace fw_nmpc {

/*
    CONSTRAINTS
*/

int constrain(const int x, const int xmin, const int xmax)
{
    return (x < xmin) ? xmin : ((x > xmax) ? xmax : x);
} // constrain

double constrain(const double x, const double xmin, const double xmax)
{
    return (x < xmin) ? xmin : ((x > xmax) ? xmax : x);
} // constrain

/*
    END CONSTRAINTS
*/

/*
    MATH
*/

void cross(double *v, const double v1[3], const double v2[3])
{
    v[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v[2] = v1[0]*v2[1] - v1[1]*v2[0];
} // cross

double dot(const double v1[3], const double v2[3])
{
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
} // dot

/*
    END MATH
*/

/*
    CONVERSIONS / TRANSFORMATIONS
*/

double calcAirDensity(const double static_pressure_pa, const double temperature_c)
{
    return static_pressure_pa / (AIR_GAS_CONST * (temperature_c - ABSOLUTE_NULL_CELSIUS));
} // calcAirDensity

void calculateSpeedStates(double *air_vel, double *ground_vel,
    double &ground_sp_sq, double &ground_sp, double &inv_ground_sp, double *unit_ground_vel,
    double &ground_sp_lat_sq, double &ground_sp_lat, double *unit_ground_vel_lat,
    double *air_polar, double *wind_vel)
{
    const double v_cos_gamma = air_polar[0] * cos(air_polar[1]);
    const double cos_xi = cos(air_polar[2]);
    const double sin_xi = sin(air_polar[2]);

    // airspeed
    air_vel[0] = v_cos_gamma * cos_xi; // v_n
    air_vel[1] = v_cos_gamma * sin_xi; // v_e
    air_vel[2] = -air_polar[0] * sin(air_polar[1]); // v_d

    // ground speed
    ground_vel[0] = air_vel[0] + wind_vel[0]; // vG_n
    ground_vel[1] = air_vel[1] + wind_vel[1]; // vG_e
    ground_vel[2] = air_vel[2] + wind_vel[2]; // vG_d
    ground_sp_lat_sq = ground_vel[0]*ground_vel[0] + ground_vel[1]*ground_vel[1]; // vG_lat^2
    ground_sp_sq = ground_sp_lat_sq + ground_vel[2]*ground_vel[2]; // vG^2
    ground_sp = sqrt(ground_sp_sq); // vG
    ground_sp_lat = sqrt(ground_sp_lat_sq); // vG_lat

    // unit ground speed
    inv_ground_sp = (ground_sp < 0.01) ? 100.0 : 1.0 / ground_sp;
    unit_ground_vel[0] = ground_vel[0] * inv_ground_sp; // vG_n unit
    unit_ground_vel[1] = ground_vel[1] * inv_ground_sp;	// vG_e unit
    unit_ground_vel[2] = ground_vel[2] * inv_ground_sp; // vG_d unit
    const double inv_ground_sp_lat = (ground_sp_lat < 0.01) ? 100.0 : 1.0 / ground_sp_lat;
    unit_ground_vel_lat[0] = ground_vel[0] * inv_ground_sp_lat; // vG_lat_n unit
    unit_ground_vel_lat[1] = ground_vel[1] * inv_ground_sp_lat;	// vG_lat_e unit
} // calculateSpeedStates

void ll2NE(double &north, double &east, const double lat, const double lon, const double lat_origin, const double lon_origin)
{
    // hopefully can get rid of this once mapping pipeline is properly referenced..
    // if not -- would be better to get this functionality from e.g. geographic lib
    // NOTE: this may not totally sync with what MAVROS is doing.. as they apparently use ECEF..

    /* convert lat/lon to N/E, about some lla origin */

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
	const double R = 6378137.0; // equatorial radius

	const double rlat0 = lat_origin*0.017453292519943;

	const double dlat = (lat-lat_origin)*0.017453292519943;
	const double dlon = (lon-lon_origin)*0.017453292519943;

	const double Rn = R/sqrt(1-(2*f-f*f)*sin(rlat0)*sin(rlat0));
	const double Rm = Rn*((1-(2*f-f*f))/(1-(2*f-f*f)*sin(rlat0)*sin(rlat0)));

	north = dlat/atan2(1,Rm);
	east = dlon/atan2(1,Rn*cos(rlat0));
} // ll2NE

Eigen::Vector2d rotate2d(const Eigen::Vector2d &vec, const double angle)
{
    const double cos_angle = cos(angle);
    const double sin_angle = sin(angle);
    return Eigen::Vector2d(vec(0) * cos_angle + vec(1) * -sin_angle, vec(0) * sin_angle + vec(1) * cos_angle);
} // rotate2d

Eigen::Vector2d rotate2d(const double sinx, const double cosx, const Eigen::Vector2d &vec)
{
    Eigen::Matrix2d rot;
    rot << cosx, -sinx, sinx, cosx;
    return rot * vec;
} // rotate2d

double unwrapHeading(const double yaw_meas, const bool reset)
{
    // TODO: use fmod

    // XXX: once slip is included.. need to disambiguate heading (xi) vs yaw notation abuse here..

    // XXX: need a catch here to reset horizon yaw states once the wrap counter gets too high

    static bool first_yaw_received = false;
    static double last_unwrapped_yaw = 0.0;
    static int wrap_counter = 0;

    if (!first_yaw_received || reset) {
      first_yaw_received = true;
      last_unwrapped_yaw = yaw_meas;
      wrap_counter = 0; // this first measurement should always be wrapped in [-pi,pi] (ATM it is coming from an atan2)
      return yaw_meas;
    }

    double delta_yaw = yaw_meas - (last_unwrapped_yaw - wrap_counter * 2.0 * M_PI); // re-wrap last yaw to find delta
    // NOTE: this assumes we aren't making more than one revolution between one update..
    //       should be a reasonable assumption..

    if ( delta_yaw < -M_PI ) {
       wrap_counter++;
       delta_yaw = delta_yaw + 2.0 * M_PI;
    }
    if ( delta_yaw > M_PI ) {
       wrap_counter--;
       delta_yaw = delta_yaw - 2.0 * M_PI;
    }

    last_unwrapped_yaw += delta_yaw;
    return last_unwrapped_yaw;
} // unwrapHeading

double wrapPi(double angle)
{
    angle = fmod(angle + M_PI, 2*M_PI);
    if (angle < 0) {
        angle += 2*M_PI;
    }

    return angle - M_PI;
} /* wrapPi */

/*
    END CONVERSIONS / TRANSFORMATIONS
*/

/*
    TERRAIN INTERPRETATION
    XXX: is "helpers" the best place for these?
*/

double getTerrainAltitude(const double pos_n, const double pos_e, const double pos_n_origin, const double pos_e_origin,
    const int map_height, const int map_width, const double map_resolution, double *terrain_map)
{
    /* look up the terrain altitude at the specified position */ // XXX: should maybe replace all with grid map functions..
    int idx_q[4];
    double dn, de;
    lookupTerrainIndex(pos_n, pos_e, pos_n_origin, pos_e_origin, map_height, map_width, map_resolution, idx_q, dn, de);

    /* bi-linear interpolation */ //XXX: should this be changed to correspond to the triangular grid used in the radial respresentation? (or provide both options and encapsulate!)
    double h12 = (1.0-dn)*terrain_map[idx_q[0]] + dn*terrain_map[idx_q[1]];
    double h34 = (1.0-dn)*terrain_map[idx_q[2]] + dn*terrain_map[idx_q[3]];
    return (1.0-de)*h12 + de*h34;
} // getTerrainAltitude

double getTerrainAltitude(double *corners, double &h12, double &h34, double &de,
    const double pos_n, const double pos_e, const double pos_n_origin, const double pos_e_origin,
    const int map_height, const int map_width, const double map_resolution, double *terrain_map)
{
    /* look up the terrain altitude at the specified position */ // XXX: should maybe replace all with grid map functions..
    int idx_q[4];
    double dn;
    lookupTerrainIndex(pos_n, pos_e, pos_n_origin, pos_e_origin, map_height, map_width, map_resolution, idx_q, dn, de);

    /* bi-linear interpolation */ //XXX: should this be changed to correspond to the triangular grid used in the radial respresentation? (or provide both options and encapsulate!)
    corners[0] = terrain_map[idx_q[0]];
    corners[1] = terrain_map[idx_q[1]];
    corners[2] = terrain_map[idx_q[2]];
    corners[3] = terrain_map[idx_q[3]];
    h12 = (1.0-dn)*corners[0] + dn*corners[1];
    h34 = (1.0-dn)*corners[2] + dn*corners[3];
    return (1.0-de)*h12 + de*h34;
} // getTerrainAltitude

void lookupTerrainIndex(const double pos_n, const double pos_e, const double pos_n_origin,
    const double pos_e_origin, const int map_height, const int map_width,
    const double map_resolution, int *idx_q, double &dn, double &de)
{
    //XXX: this could very likely be replaced with grid map primitives

    const int map_height_1 = map_height - 1;
    const int map_width_1 = map_width - 1;

    /* relative position / indices */
    const double rel_n = pos_n - pos_n_origin;
    const double rel_n_bar = rel_n / map_resolution;
    int idx_n = (int)(floor(rel_n_bar));
    const double rel_e = pos_e - pos_e_origin;
    const double rel_e_bar = rel_e / map_resolution;
    int idx_e = (int)(floor(rel_e_bar));

    /* interpolation weights */
    dn = rel_n_bar-idx_n;
    de = rel_e_bar-idx_e;

    /* cap ends */
    if (idx_n < 0) {
        idx_n = 0;
    }
    else if (idx_n > map_height_1) {
        idx_n = map_height_1;
    }
    if (idx_e < 0) {
        idx_e = 0;
    }
    else if (idx_e > map_width_1) {
        idx_e = map_width_1;
    }

    /* neighbors (north) */
    int q_n[4];
    if (idx_n >= map_height_1) {
        q_n[0] = map_height_1;
        q_n[1] = map_height_1;
        q_n[2] = map_height_1;
        q_n[3] = map_height_1;
    }
    else {
        q_n[0] = idx_n;
        q_n[1] = idx_n + 1;
        q_n[2] = idx_n;
        q_n[3] = idx_n + 1;
    }
    /* neighbors (east) */
    int q_e[4];
    if (idx_e >= map_width_1) {
        q_e[0] = map_width_1;
        q_e[1] = map_width_1;
        q_e[2] = map_width_1;
        q_e[3] = map_width_1;
    }
    else {
        q_e[0] = idx_e;
        q_e[1] = idx_e;
        q_e[2] = idx_e + 1;
        q_e[3] = idx_e + 1;
    }

    /* neighbors row-major indices */
    idx_q[0] = q_n[0]*map_width + q_e[0];
    idx_q[1] = q_n[1]*map_width + q_e[1];
    idx_q[2] = q_n[2]*map_width + q_e[2];
    idx_q[3] = q_n[3]*map_width + q_e[3];
} // lookupTerrainIndex

/*
    END TERRAIN INTERPRETATION
*/

} // namespace fw_nmpc
