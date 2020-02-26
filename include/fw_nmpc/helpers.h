#ifndef HELPERS_H_
#define HELPERS_H_

#include <Eigen/Eigen>

namespace fw_nmpc {

const double DEG_TO_RAD = 0.017453292519943;    // degrees to radians XXX: is there really no other function already built in for this?
const double AIR_GAS_CONST = 287.1;             // [J/(kg*K)]
const double ABSOLUTE_NULL_CELSIUS = -273.15;   // [C]
const double ONE_G = 9.81;                      // [m/s^2]

/*
 * compilation of helpful functions common to fw_nmpc
 */

// constraints
int constrain(const int x, const int xmin, const int xmax);
double constrain(const double x, const double xmin, const double xmax);

// math
void cross(double *v, const double v1[3], const double v2[3]);
double dot(const double v1[3], const double v2[3]);
void renormalizeUnitVelocity(Eigen::Vector3d &unit_vel);

// conversions / transformations
double calcAirDensity(const double static_pressure_pa, const double temperature_c);
void calculateSpeedStates(double *air_vel, double *ground_vel,
    double &ground_sp_sq, double &ground_sp, double &inv_ground_sp, double *unit_ground_vel,
    double &ground_sp_lat_sq, double &ground_sp_lat, double *unit_ground_vel_lat,
    double *air_polar, double *wind_vel);
void ll2NE(double &north, double &east, const double lat, const double lon, const double lat_origin, const double lon_origin);
double unwrapHeading(const double yaw_meas, const bool reset);

// terrain interpretation functions
double getTerrainAltitude(const double pos_n, const double pos_e, const double pos_n_origin, const double pos_e_origin,
    const int map_height, const int map_width, const double map_resolution, double *terrain_map);
double getTerrainAltitude(double *corners, double &h12, double &h34, double &de,
    const double pos_n, const double pos_e, const double pos_n_origin, const double pos_e_origin,
    const int map_height, const int map_width, const double map_resolution, double *terrain_map);
void lookupTerrainIndex(const double pos_n, const double pos_e, const double pos_n_origin, const double pos_e_origin,
    const int map_height, const int map_width, const double map_resolution, int *idx_q, double &dn, double &de);

} // namespace fw_nmpc

#endif // HELPERS_H_
