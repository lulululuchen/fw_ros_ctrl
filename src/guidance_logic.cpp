#include <fw_nmpc/guidance_logic.h>
#include <fw_nmpc/helpers.h>
#include <math.h>

namespace fw_nmpc {

GuidanceLogic::GuidanceLogic() :
    fix_vert_pos_err_bnd_(true),
    gamma_app_max_(0.3),
    guidance_sel_(GuidanceLogicTypes::PW_QUAD),
    T_lat_(5.0),
    T_lon_(5.0),
    unit_z_app_max_(0.3),
    use_occ_as_guidance_(false),
    vert_pos_err_bnd_(5.0)
{} // GuidanceLogic

void GuidanceLogic::setFPAAppMax(const double fpa)
{
    gamma_app_max_ = fpa;
    unit_z_app_max_ = sin(gamma_app_max_);
} // setFPAAppMax

void GuidanceLogic::augmentTerrainCostToGuidance(Eigen::Vector3d &unit_ground_vel_sp, const Eigen::Vector3d &unit_terrain_grad, const double inv_terrain_prio)
{
    // TODO: naming.. also change gradient to eigen vector3d. could also consider renaming the function itself..

    // unit terrain position gradient
    const double mult = (unit_terrain_grad.hasNaN()) ? 0.0 : -1.0; // if non-zero, negate the gradient

    // augment unit vel sp //XXX: should this change the reference, or is it better to have a copy output?
    unit_ground_vel_sp *= inv_terrain_prio; // deprioritize
    unit_ground_vel_sp += mult * unit_terrain_grad * (1.0 - inv_terrain_prio); // augment the negative terrain gradient
} // augmentTerrainCostToGuidance

void GuidanceLogic::calculateUnitVelocityReference(Eigen::Vector3d &unit_ground_vel_sp, double &err_lat, double &err_lon, double &unit_err_lat,
    const Eigen::Vector3d &veh_pos, const Eigen::Vector3d &ground_vel, const double ground_sp_lat, const double terrain_alt,
    const ManualControlSetpoint &manual_control_sp, const PathSetpoint &path_sp)
{
    /*
        guidance logic class interface
    */

    // guidance logic
    if (manual_control_sp.enabled) {
        // we are receiving manual setpoints from the remote
        followManual(unit_ground_vel_sp, err_lat, err_lon, unit_err_lat, manual_control_sp, terrain_alt, veh_pos, ground_vel, ground_sp_lat);
    }
    else {
        // full auto - mpc is following some input path

        // use guidance logic to follow path
        followPath(unit_ground_vel_sp, err_lat, err_lon, unit_err_lat, path_sp, veh_pos, ground_vel, ground_sp_lat);
    }
} // calculateUnitVelocityReference

double GuidanceLogic::calculatePositionErrorBoundary(const double v, const double tc)
{
    /*
     * calculates the metric error boundary normal to the closest point/tangent
     * on the path. near zero vehicle speeds are smoothly accounted for by
     * quadratically converging the boundary to a fixed value at zero speed.
     *
     * inputs:
     * v: vehicle speed (magnitude) [m/s]
     * tc: guidance logic time constant [s]
     * outputs:
     * err_b: error boundary [m]
     */
    double err_b;
    if (v < 1.0) {
        err_b = tc * 0.5 * (1.0 + v*v);
    }
    else {
        err_b = tc * v;
    }
    return err_b;
} // calculatePositionErrorBoundary

void GuidanceLogic::calculateReferencePoseOnLine(Eigen::Vector3d &closest_pt_on_path, Eigen::Vector3d &unit_path_tangent,
    const Eigen::Vector3d &pt_on_line, const double bearing, const double fpa_ref, const Eigen::Vector3d& veh_pos)
{
    /* Calculate reference position and direction on a line path
     *
     * inputs:
     * pt_on_line (3,1): arbitrary point on line, NED [m]
     * bearing: path bearing at pt_on_line [rad]
     * fpa_ref: path flight path angle at pt_on_line [rad]
     * veh_pos (3,1): vehicle position, NED [m]
     * veh_vel (3,1): vehicle velocity, NED [m/s]
     *
     * outputs:
     * closest_pt_on_path (3,1): "closest" point on line path [m]
     * unit_path_tangent (3,1): unit path tangent vector [~]
     */

    // lateral-directional (2d) unit path tangent components
    const double tp_lat_n = cos(bearing);
    const double tp_lat_e = sin(bearing);

    // "closest" point on track
    const double br_n = veh_pos(0) - pt_on_line(0);
    const double br_e = veh_pos(1) - pt_on_line(1);
    const double tp_dot_br = tp_lat_n * br_n + tp_lat_e * br_e;
    const double proj_br_tp_n = tp_dot_br * tp_lat_n;
    const double proj_br_tp_e = tp_dot_br * tp_lat_e;
    const double dist_lat = proj_br_tp_n * tp_lat_n + proj_br_tp_e * tp_lat_e;
    closest_pt_on_path(0) = pt_on_line(0) + proj_br_tp_n;
    closest_pt_on_path(1) = pt_on_line(1) + proj_br_tp_e;
    closest_pt_on_path(2) = pt_on_line(2) - dist_lat * tan(fpa_ref);

    // unit path tangent vector
    const double cos_fpa = cos(fpa_ref);
    unit_path_tangent(0) = tp_lat_n*cos_fpa;
    unit_path_tangent(1) = tp_lat_e*cos_fpa;
    unit_path_tangent(2) = -sin(fpa_ref);
} // calculateReferencePoseOnLine

void GuidanceLogic::calculateReferencePoseOnLoiter(Eigen::Vector3d &closest_pt_on_path, Eigen::Vector3d &unit_path_tangent,
    const Eigen::Vector3d &circle_center, const double signed_radius, const Eigen::Vector3d& veh_pos, const Eigen::Vector3d& veh_vel, const double v_lat)
{
    // loiter direction
    const double loiter_dir = (signed_radius < 0.0) ? -1.0 : 1.0;
    const double radius = (fabs(signed_radius) < 0.1) ? 0.1 : fabs(signed_radius);

    // vector from circle center to aircraft
    const double br_n = veh_pos(0)-circle_center(0);
    const double br_e = veh_pos(1)-circle_center(1);
    const double br_d = veh_pos(2)-circle_center(2);

    // lateral-directional distance to circle center
    const double dist_to_center = sqrt(br_n*br_n + br_e*br_e);

    double br_n_unit, br_e_unit;
    double p_n, p_e;
    if (dist_to_center < 0.1) {
        if (v_lat < 0.1) {
            // arbitrarily set the point in the northern top of the circle
            br_n_unit = 1.0;
            br_e_unit = 0.0;

            // closest point on circle
            p_n = circle_center(0) + br_n_unit * radius;
            p_e = circle_center(1) + br_e_unit * radius;
        }
        else {
            // set the point in the direction we are moving
            br_n_unit = veh_vel(0) / v_lat * 0.1;
            br_e_unit = veh_vel(1) / v_lat * 0.1;

            // closest point on circle
            p_n = circle_center(0) + br_n_unit * radius;
            p_e = circle_center(1) + br_e_unit * radius;
        }
    }
    else {
        // set the point in the direction of the aircraft
        br_n_unit = br_n / dist_to_center;
        br_e_unit = br_e / dist_to_center;

        // closest point on circle
        p_n = circle_center(0) + br_n_unit * radius;
        p_e = circle_center(1) + br_e_unit * radius;
    }

    // path tangent unit vector
    const double tp_lat_n = -br_e_unit * loiter_dir;
    const double tp_lat_e = br_n_unit * loiter_dir;

    // "closest" point on path
    closest_pt_on_path(0) = p_n;
    closest_pt_on_path(1) = p_e;
    closest_pt_on_path(2) = circle_center(2);

    // unit path tangent vector
    unit_path_tangent(0) = tp_lat_n;
    unit_path_tangent(1) = tp_lat_e;
    unit_path_tangent(2) = 0.0;
} // calculateReferencePoseOnLoiter

/* generate unit velocity commands to follow manual setpoints */
void GuidanceLogic::followManual(Eigen::Vector3d &unit_ground_vel_sp, double &err_lat, double &err_lon, double &unit_err_lat,
    const ManualControlSetpoint &manual_control_sp, const double terr_alt, const Eigen::Vector3d &veh_pos,
    const Eigen::Vector3d &ground_vel, const double &ground_sp_lat)
{
    if (manual_control_sp.type == ManualControlTypes::DIRECTION) {
        // track the commanded unit ground velocity
        unit_ground_vel_sp = manual_control_sp.unit_vel;
    }
    else if (manual_control_sp.type == ManualControlTypes::ALTITUDE) {
        // track manual bearing and altitude setpoints

        // set xy vel
        unit_ground_vel_sp.segment(0,2) = manual_control_sp.unit_vel.segment(0,2);

        // use guidance to calculate flight path angle setpoint
        if (guidance_sel_ == GuidanceLogicTypes::ARCTAN) {
            // arctangent vector field guidance
            unit_ground_vel_sp(2) = longitudinalArctanVFGuidance(err_lon, -manual_control_sp.alt, 0.0, 0.0, veh_pos(2), ground_vel(2));
        }
        else { // default = GuidanceLogicTypes::PW_QUAD
            // piece-wise quadratic guidance
            unit_ground_vel_sp(2) = longitudinalPwQuadGuidance(err_lon, -manual_control_sp.alt, 0.0, veh_pos(2), ground_vel(2));
        }

        // re-normalize decoupled (lat/lon) unit velocity
        renormalizeUnitVelocity(unit_ground_vel_sp);
    }
    else if (manual_control_sp.type == ManualControlTypes::TERRAIN_ALTITUDE) {
        // track manual bearing and relative altitude setpoints

        // set xy vel
        unit_ground_vel_sp.segment(0,2) = manual_control_sp.unit_vel.segment(0,2);

        // set absolute altitude based on node-wise terrain height
        double alt = terr_alt + manual_control_sp.rel_alt;

        // terrain slope
        // XXX: for now flat.. could potentially calculate the slope based on the current course .. it is calculated anyway internally in the cost jacobian.
        const double unit_path_tangent_d = 0.0;

        // use guidance to calculate flight path angle setpoint
        if (guidance_sel_ == GuidanceLogicTypes::ARCTAN) {
            // arctangent vector field guidance
            unit_ground_vel_sp(2) = longitudinalArctanVFGuidance(err_lon, -alt, unit_path_tangent_d, 0.0, veh_pos(2), ground_vel(2));
        }
        else { // default = GuidanceLogicTypes::PW_QUAD
            // piece-wise quadratic guidance
            unit_ground_vel_sp(2) = longitudinalPwQuadGuidance(err_lon, -alt, unit_path_tangent_d, veh_pos(2), ground_vel(2));
        }

        // re-normalize decoupled (lat/lon) unit velocity
        renormalizeUnitVelocity(unit_ground_vel_sp);
    }
    // else {
    //     // ...
    // }
} // followManual

/* generate unit velocity commands to follow the reference path */
void GuidanceLogic::followPath(Eigen::Vector3d &unit_ground_vel_sp, double &err_lat, double &err_lon, double &unit_err_lat,
    const PathSetpoint &path_sp, const Eigen::Vector3d &veh_pos, const Eigen::Vector3d &veh_vel, const double ground_sp_lat)
{
    // init
    Eigen::Vector3d closest_pt_on_path;
    Eigen::Vector3d unit_path_tangent;

    if (path_sp.type == PathTypes::LOITER) {
        /* loiter at fixed altitude */

        // get reference pose at closest point on loiter circle
        calculateReferencePoseOnLoiter(closest_pt_on_path, unit_path_tangent, path_sp.pos, path_sp.signed_radius, veh_pos, veh_vel, ground_sp_lat);

        // guidance logic
        if (guidance_sel_ == GuidanceLogicTypes::ARCTAN) {
            // arctangent vector field guidance
            arctanVFGuidance(unit_ground_vel_sp, err_lat, err_lon, unit_err_lat,
                closest_pt_on_path, unit_path_tangent, path_sp.bearing, path_sp.fpa,
                veh_pos, veh_vel, ground_sp_lat);
        }
        else { // default or PW_QUAD
            // piece-wise quadratic guidance
            pwQuadGuidance(unit_ground_vel_sp, err_lat, err_lon, unit_err_lat,
                closest_pt_on_path, unit_path_tangent, veh_pos, veh_vel, ground_sp_lat);
        }
    }
    else if (path_sp.type == PathTypes::LINE) {
        /* 3D line */

        // get reference pose at closest point on line
        calculateReferencePoseOnLine(closest_pt_on_path, unit_path_tangent, path_sp.pos, path_sp.bearing, path_sp.fpa, veh_pos);

        // guidance logic
        if (guidance_sel_ == GuidanceLogicTypes::ARCTAN) {
            // arctangent vector field guidance
            arctanVFGuidance(unit_ground_vel_sp, err_lat, err_lon, unit_err_lat,
                closest_pt_on_path, unit_path_tangent, path_sp.bearing, path_sp.fpa,
                veh_pos, veh_vel, ground_sp_lat);
        }
        else { // default or PW_QUAD
            // piece-wise quadratic guidance
            pwQuadGuidance(unit_ground_vel_sp, err_lat, err_lon, unit_err_lat,
                closest_pt_on_path, unit_path_tangent, veh_pos, veh_vel, ground_sp_lat);
        }
    }
    else {
        /* unknown */

        // position error
        err_lat = 0.0;
        err_lon = 0.0;

        // unit track error
        unit_err_lat = 0.0;

        // unit ground velocity setpoint -- fly north..
        unit_ground_vel_sp(0) = 1.0;
        unit_ground_vel_sp(1) = 0.0;
        unit_ground_vel_sp(2) = 0.0;
    }
} // followPath

//TODO: make class for guidance, remove redundancy
double GuidanceLogic::longitudinalPwQuadGuidance(double &err_lon, const double d_ref, const double unit_path_tangent_d, const double veh_pos_d, const double veh_vel_d)
{
    /* Piecewise Quadratic Guidance Logic
     *
     * inputs:
     * d_ref (3,1): down reference [m]
     * unit_path_tangent_d: unit down path tangent at d_ref [~]
     * veh_pos_d: vehicle position, NED [m]
     * veh_vel_d: vehicle down velocity, D [m/s]
     *
     * outputs:
     * err_lon: longitudinal (vertical) signed track-error [m]
     *
     * return:
     * unit_vel_sp_d: vehicle unit velocity setpoint [~]
     */

    // position error
    err_lon = d_ref - veh_pos_d;

    // longitudinal speed
    const double v_lon = fabs(veh_vel_d);

    // track-error boundaries
    double err_b_lon;
    if (fix_vert_pos_err_bnd_) {
        err_b_lon = vert_pos_err_bnd_;
    }
    else {
        err_b_lon = calculatePositionErrorBoundary(v_lon, T_lon_); // longitudinal track-error boundary
    }

    // unit track error
    const double unit_err_lon = constrain(fabs(err_lon)/err_b_lon, 0.0, 1.0);

    // quadratic transition variables
    const double thetal_lon = -unit_err_lon*(unit_err_lon - 2.0);

    // sign of longitudinal position error
    const double sign_e_lon = (err_lon < 0.0) ? -1.0 : 1.0;

    // unit z velocity setpoint
    return (1.0 - thetal_lon) * unit_path_tangent_d + thetal_lon * unit_z_app_max_ * sign_e_lon;
} // longitudinalPwQuadGuidance

//TODO: make class for guidance, remove redundancy
double GuidanceLogic::longitudinalArctanVFGuidance(double &err_lon,
    const double d_ref, const double unit_path_tangent_d, const double fpa_ref, const double veh_pos_d, const double veh_vel_d)
{
    /*
     * inputs:
     * d_ref (3,1): down reference [m]
     * unit_path_tangent_d: unit down path tangent at d_ref [~]
     * fpa_ref: path flight path angle at pt_on_path [rad] (redundant, but saves on trig evals)
     * veh_pos_d: vehicle position, NED [m]
     * veh_vel_d: vehicle down velocity, D [m/s]
     *
     * outputs:
     * err_lon: longitudinal (vertical) signed track-error [m]
     *
     * return:
     * unit_vel_sp_d: vehicle unit velocity setpoint [~]
     */

     // position error
     err_lon = d_ref - veh_pos_d;

     // longitudinal speed
     const double v_lon = fabs(veh_vel_d);

     // track-error boundaries
     double err_b_lon;
     if (fix_vert_pos_err_bnd_) {
         err_b_lon = vert_pos_err_bnd_;
     }
     else {
         err_b_lon = calculatePositionErrorBoundary(v_lon, T_lon_); // longitudinal track-error boundary
     }

     // unit track-error
     const double unit_err_lon = constrain(fabs((err_lon)/err_b_lon), 0.0, 1.0);

     // flight path approach angle
     const double Gamma_app = -gamma_app_max_ * atan(M_PI_2*unit_err_lon);

     // normalized ground velocity setpoint
     return -sin(fpa_ref + Gamma_app);
} // longitudinalArctanVFGuidance

void GuidanceLogic::arctanVFGuidance(Eigen::Vector3d &unit_vel_sp, double &err_lat, double &err_lon, double &unit_err_lat,
    const Eigen::Vector3d &pt_on_path, const Eigen::Vector3d &unit_path_tangent, const double bearing, const double fpa_ref,
    const Eigen::Vector3d &veh_pos, const Eigen::Vector3d &veh_vel, const double v_lat)
{
    /* Arctangent Vector-field Guidance Logic
     *
     * inputs:
     * pt_on_path (3,1): point on path, NED [m]
     * unit_path_tangent (3,1): unit path tangent at pt_on_path [~]
     * bearing: path bearing at pt_on_path [rad] (redundant, but saves on trig evals)
     * fpa_ref: path flight path angle at pt_on_path [rad] (redundant, but saves on trig evals)
     * veh_pos (3,1): vehicle position, NED [m]
     * veh_vel (3,1): vehicle velocity, NED [m/s]
     * v_lat: vehicle lateral-directional (horizontal) speed [m/s] (redundant, but saves a couple divisions)
     *
     * outputs:
     * unit_vel_sp (3,1): vehicle unit velocity setpoint [~]
     * err_lat: lateral-directional (horizontal) signed track-error [m]
     * err_lon: longitudinal (vertical) signed track-error [m]
     * unit_err_lat: normalized lateral-directional (horizontal) track-error [~]
     *
     * returns:
     * unit_vel_sp (3,1): vehicle unit velocity setpoint [~]
     */

    // lateral-directional path tangent components
    const double norm_unit_path_tangent_lat = unit_path_tangent.segment(0,2).norm();
    const double tp_n = unit_path_tangent(0) / norm_unit_path_tangent_lat;
    const double tp_e = unit_path_tangent(1) / norm_unit_path_tangent_lat;

    // position error
    err_lat = (veh_pos(0)-pt_on_path(0))*tp_e - (veh_pos(1)-pt_on_path(1))*tp_n;
    err_lon = pt_on_path(2) - veh_pos(2);

    // longitudinal speed
    const double v_lon = fabs(veh_vel(2));

    // track-error boundaries
    const double err_b_lat = calculatePositionErrorBoundary(v_lat, T_lat_); // lateral-directional track-error boundary
    double err_b_lon;
    if (fix_vert_pos_err_bnd_) {
        err_b_lon = vert_pos_err_bnd_;
    }
    else {
        err_b_lon = calculatePositionErrorBoundary(v_lon, T_lon_); // longitudinal track-error boundary
    }

    // unit track-error
    unit_err_lat = constrain(fabs((err_lat)/err_b_lat), 0.0, 1.0);
    const double unit_err_lon = constrain(fabs((err_lon)/err_b_lon), 0.0, 1.0);

    // course approach angle
    const double chi_app = atan(M_PI_2*unit_err_lat);

    // flight path approach angle
    const double Gamma_app = -gamma_app_max_ * atan(M_PI_2*unit_err_lon);

    // normalized ground velocity setpoint
    const double cos_gamma = cos(fpa_ref + Gamma_app);
    unit_vel_sp(0) = cos_gamma*cos(bearing + chi_app);
    unit_vel_sp(1) = cos_gamma*sin(bearing + chi_app);
    unit_vel_sp(2) = -sin(fpa_ref + Gamma_app);
} // arctanVFGuidance

void GuidanceLogic::pwQuadGuidance(Eigen::Vector3d &unit_vel_sp, double &err_lat, double &err_lon, double &unit_err_lat,
    const Eigen::Vector3d& pt_on_path, const Eigen::Vector3d& unit_path_tangent, const Eigen::Vector3d& veh_pos, const Eigen::Vector3d& veh_vel, const double v_lat)
{
    /* Piecewise Quadratic Guidance Logic
     *
     * inputs:
     * pt_on_path (3,1): closet point on path, NED [m]
     * unit_path_tangent (3,1): unit path tangent at pt_on_path [~]
     * veh_pos (3,1): vehicle position, NED [m]
     * veh_vel (3,1): vehicle velocity, NED [m/s]
     * v_lat: vehicle lateral-directional (horizontal) speed [m/s] (redundant, but saves a couple divisions)
     *
     * outputs:
     * unit_vel_sp (3,1): vehicle unit velocity setpoint [~]
     * err_lat: lateral-directional (horizontal) signed track-error [m]
     * unit_err_lat: normalized lateral-directional (horizontal) track-error [~]
     * err_lon: longitudinal (vertical) signed track-error [m]
     */

    // lateral-directional path tangent components
    Eigen::Vector2d unit_path_tangent_lat = unit_path_tangent.segment(0,2).normalized();

    // lateral-directional position error
    Eigen::Vector2d pos_err_lat = veh_pos.segment(0,2) - pt_on_path.segment(0,2);

    // position error
    err_lat = pos_err_lat(0) * unit_path_tangent_lat(1) - pos_err_lat(1) * unit_path_tangent_lat(0);
    err_lon = pt_on_path(2) - veh_pos(2);

    // longitudinal speed
    const double v_lon = fabs(veh_vel(2));

    // track-error boundaries
    const double err_b_lat = calculatePositionErrorBoundary(v_lat, T_lat_); // lateral-directional track-error boundary
    double err_b_lon;
    if (fix_vert_pos_err_bnd_) {
        err_b_lon = vert_pos_err_bnd_;
    }
    else {
        err_b_lon = calculatePositionErrorBoundary(v_lon, T_lon_); // longitudinal track-error boundary
    }

    // unit track error
    unit_err_lat = constrain(fabs(err_lat)/err_b_lat, 0.0, 1.0);
    const double unit_err_lon = constrain(fabs(err_lon)/err_b_lon, 0.0, 1.0);

    // quadratic transition variables
    const double thetal_lat = -unit_err_lat*(unit_err_lat - 2.0);
    const double thetal_lon = -unit_err_lon*(unit_err_lon - 2.0);

    if (fabs(err_lat) > 0.00001) {
        pos_err_lat.normalize(); // normalized from this point on in function
    }
    const double sign_e_lon = (err_lon < 0.0) ? -1.0 : 1.0;

    // unit ground velocity setpoint
    unit_vel_sp(2) = (1.0 - thetal_lon) * unit_path_tangent(2) + thetal_lon * unit_z_app_max_ * sign_e_lon;
    unit_vel_sp.segment(0,2) = ((1.0 - thetal_lat) * unit_path_tangent_lat + thetal_lat * -pos_err_lat) * (1.0-unit_vel_sp(2)*unit_vel_sp(2)); // normalize xy considering z
} // pwQuadGuidance

} // namespace fw_nmpc
