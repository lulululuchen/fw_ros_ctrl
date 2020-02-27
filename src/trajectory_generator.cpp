#include <fw_nmpc/trajectory_generator.h>
#include <fw_nmpc/common/helpers.h>
#include <math.h>

namespace fw_nmpc {

TrajectoryGenerator::TrajectoryGenerator() :
    mc_was_disabled_(true),
    mc_lon_mode_changed_(true),
    mc_lat_mode_changed_(true),
    mc_last_lon_mode_(MCLongitudinalModes::FPA),
    mc_last_lat_mode_(MCLateralModes::ROLL),
    heading_hold_(0.0),
    alt_hold_(100.0),
    max_delta_heading_(0.5),
    max_delta_alt_(5.0),
    dt_(0.1),
    cross_err_thres_(0.1),
    track_err_thres_(0.1),
    tg_mode_(TGMode::GUIDE_TO_PATH),
    en_off_track_roll_ff_(false)
{} // TrajectoryGenerator

/*
    GUIDANCE PARAMETERS
*/

void TrajectoryGenerator::setNPFGParams(bool en_min_ground_speed, const double min_ground_speed_g, const double min_ground_speed_e_max,
    const double nte_fraction, const double p_gain, const double time_const, const double wind_ratio_buf)
{
    npfg_.enableBackwardsSolution(false);
    npfg_.enableMinGroundSpeed(en_min_ground_speed);
    npfg_.enableTrackKeeping(true);
    npfg_.enableWindExcessRegulation(true);
    npfg_.setMinGroundSpeed(min_ground_speed_g);
    npfg_.setMinGroundSpeedEMax(min_ground_speed_e_max);
    npfg_.setNTEFraction(nte_fraction);
    npfg_.setPGain(p_gain);
    npfg_.setTimeConstant(time_const);
    npfg_.setWindRatioBuf(wind_ratio_buf);
} // setNPFGParams

void TrajectoryGenerator::setPWQGParams(const double fix_vert_pos_err_bnd, const double time_const, const double fpa_app_max, const double vert_pos_err_bnd)
{
    pwqg_.setFixedVertPosErrBound(fix_vert_pos_err_bnd);
    pwqg_.setTimeConstant(time_const);
    pwqg_.setVertPosErrBound(vert_pos_err_bnd);
    pwqg_.setFPAAppMax(fpa_app_max);
} // setPWQGParams

/*
    END GUIDANCE PARAMETERS
*/

/*
    PATH FOLLOWING CALCULATIONS
*/

double TrajectoryGenerator::calcRefPoseOnLine(Eigen::Vector3d &closest_pt_on_path, const Eigen::Vector3d &pt_on_line,
    const Eigen::Vector3d &unit_dir_vec, const Eigen::Vector3d& veh_pos)
{
    /*
        Calculate reference position and direction on a line path

        inputs:
        pt_on_line (3,1): arbitrary point on line, NED [m]
        unit_dir_vec (3,1): line direction, NED [~]
        veh_pos (3,1): vehicle position, NED [m]

        outputs:
        closest_pt_on_path (3,1): "closest" point on line path [m]
        unit_path_tangent (3,1): unit path tangent vector [~]

        return:
        signed lateral-directional distance to line
    */

    // lateral-directional (2d) unit path tangent components
    const double one_minus_z2 = 1.0 - unit_dir_vec(2) * unit_dir_vec(2);
    const double tp_lat_n = unit_dir_vec(0) * one_minus_z2;
    const double tp_lat_e = unit_dir_vec(1) * one_minus_z2;

    // "closest" point on track
    const double br_n = veh_pos(0) - pt_on_line(0);
    const double br_e = veh_pos(1) - pt_on_line(1);
    const double tp_dot_br = tp_lat_n * br_n + tp_lat_e * br_e;
    const double proj_br_tp_n = tp_dot_br * tp_lat_n;
    const double proj_br_tp_e = tp_dot_br * tp_lat_e;
    const double dist_lat = proj_br_tp_n * tp_lat_n + proj_br_tp_e * tp_lat_e;
    closest_pt_on_path(0) = pt_on_line(0) + proj_br_tp_n;
    closest_pt_on_path(1) = pt_on_line(1) + proj_br_tp_e;
    closest_pt_on_path(2) = pt_on_line(2) - dist_lat * unit_dir_vec(2) / one_minus_z2;

    return tp_lat_n * br_e - tp_lat_e * br_n;
} // calcRefPoseOnLine

double TrajectoryGenerator::calcRefPoseOnLine2D(Eigen::Vector2d &closest_pt_on_path, const Eigen::Vector2d &pt_on_line,
    const Eigen::Vector2d &unit_dir_vec, const Eigen::Vector2d& veh_pos)
{
    /*
        Calculate reference position and direction on a 2D line path

        inputs:
        pt_on_line (2,1): arbitrary point on line, NE [m]
        unit_dir_vec (2,1): line direction, NE [~]
        veh_pos (2,1): vehicle position, NE [m]

        outputs:
        closest_pt_on_path (2,1): "closest" point on line path [m]
        unit_path_tangent (2,1): unit path tangent vector [~]

        return:
        signed distance to line
     */

    // "closest" point on track
    const double br_n = veh_pos(0) - pt_on_line(0);
    const double br_e = veh_pos(1) - pt_on_line(1);
    const double tp_dot_br = unit_dir_vec(0) * br_n + unit_dir_vec(1) * br_e;
    const double proj_br_tp_n = tp_dot_br * unit_dir_vec(0);
    const double proj_br_tp_e = tp_dot_br * unit_dir_vec(1);
    closest_pt_on_path(0) = pt_on_line(0) + proj_br_tp_n;
    closest_pt_on_path(1) = pt_on_line(1) + proj_br_tp_e;

    return br_e * unit_dir_vec(0) - br_n * unit_dir_vec(1);
} // calcRefPoseOnLine2D

double TrajectoryGenerator::calcRefPoseOnLoiter(Eigen::Vector3d &closest_pt_on_path, Eigen::Vector3d &unit_path_tangent,
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
    if (dist_to_center < 0.1) {
        if (v_lat < 0.1) {
            // arbitrarily set the point in the northern top of the circle
            br_n_unit = 1.0;
            br_e_unit = 0.0;

            // closest point on circle
            closest_pt_on_path(0) = circle_center(0) + br_n_unit * radius;
            closest_pt_on_path(1) = circle_center(1) + br_e_unit * radius;
        }
        else {
            // set the point in the direction we are moving
            br_n_unit = veh_vel(0) / v_lat * 0.1;
            br_e_unit = veh_vel(1) / v_lat * 0.1;

            // closest point on circle
            closest_pt_on_path(0) = circle_center(0) + br_n_unit * radius;
            closest_pt_on_path(1) = circle_center(1) + br_e_unit * radius;
        }
    }
    else {
        // set the point in the direction of the aircraft
        br_n_unit = br_n / dist_to_center;
        br_e_unit = br_e / dist_to_center;

        // closest point on circle
        closest_pt_on_path(0) = circle_center(0) + br_n_unit * radius;
        closest_pt_on_path(1) = circle_center(1) + br_e_unit * radius;
    }
    closest_pt_on_path(2) = circle_center(2);

    // path tangent unit vector
    const double tp_lat_n = -br_e_unit * loiter_dir;
    const double tp_lat_e = br_n_unit * loiter_dir;

    // unit path tangent vector
    unit_path_tangent(0) = tp_lat_n;
    unit_path_tangent(1) = tp_lat_e;
    unit_path_tangent(2) = 0.0;

    return (closest_pt_on_path(1) - veh_pos(1)) * tp_lat_n - (closest_pt_on_path(0) - veh_pos(0)) * tp_lat_e;
} // calcRefPoseOnLoiter

double TrajectoryGenerator::calcRefPoseOnLoiter2D(Eigen::Vector2d &closest_pt_on_path, Eigen::Vector2d &unit_path_tangent,
    const Eigen::Vector2d &circle_center, const double signed_radius, const Eigen::Vector2d& veh_pos, const Eigen::Vector2d& veh_vel, const double veh_speed)
{
    // loiter direction
    const double loiter_dir = (signed_radius < 0.0) ? -1.0 : 1.0;
    const double radius = (fabs(signed_radius) < 0.1) ? 0.1 : fabs(signed_radius);

    // vector from circle center to aircraft
    const double br_n = veh_pos(0)-circle_center(0);
    const double br_e = veh_pos(1)-circle_center(1);

    // distance to circle center
    const double dist_to_center = sqrt(br_n*br_n + br_e*br_e);

    double br_n_unit, br_e_unit;
    if (dist_to_center < 0.1) {
        if (veh_speed < 0.1) {
            // arbitrarily set the point in the northern top of the circle
            br_n_unit = 1.0;
            br_e_unit = 0.0;

            // closest point on circle
            closest_pt_on_path(0) = circle_center(0) + br_n_unit * radius;
            closest_pt_on_path(1) = circle_center(1) + br_e_unit * radius;
        }
        else {
            // set the point in the direction we are moving
            br_n_unit = veh_vel(0) / veh_speed * 0.1;
            br_e_unit = veh_vel(1) / veh_speed * 0.1;

            // closest point on circle
            closest_pt_on_path(0) = circle_center(0) + br_n_unit * radius;
            closest_pt_on_path(1) = circle_center(1) + br_e_unit * radius;
        }
    }
    else {
        // set the point in the direction of the aircraft
        br_n_unit = br_n / dist_to_center;
        br_e_unit = br_e / dist_to_center;

        // closest point on circle
        closest_pt_on_path(0) = circle_center(0) + br_n_unit * radius;
        closest_pt_on_path(1) = circle_center(1) + br_e_unit * radius;
    }

    // path tangent unit vector
    unit_path_tangent(0) = -br_e_unit * loiter_dir;
    unit_path_tangent(1) = br_n_unit * loiter_dir;

    return (closest_pt_on_path(1) - veh_pos(1)) * unit_path_tangent(0) - (closest_pt_on_path(0) - veh_pos(0)) * unit_path_tangent(1);
} // calcRefPoseOnLoiter2D

/*
    END PATH FOLLOWING CALCULATIONS
*/

/*
    TRAJECTORY GENERATION
*/

void TrajectoryGenerator::genTrajectoryToPath2D(Eigen::Ref<Eigen::MatrixXd> pos_traj, Eigen::Ref<Eigen::MatrixXd> airsp_traj,
    Eigen::Ref<Eigen::MatrixXd> heading_traj, Eigen::Ref<Eigen::MatrixXd> roll_traj, const int len_traj,
    Eigen::Vector2d pos_0, const Eigen::Vector2d &ground_vel_0, const double airspeed_0, const double heading_0, const Eigen::Vector2d &wind_vel,
    const double airspeed_nom, const double airspeed_max, const PathSetpoint &path_sp, const double roll_lim_rad)
{
    // check that we are not in the circle center
    if (path_sp.type == PathTypes::LOITER) {
        Eigen::Vector2d radial_dist = pos_0 - path_sp.pos.segment(0,2);
        if (radial_dist.norm() < 0.1) {
            pos_0 = path_sp.pos.segment(0,2) + Eigen::Vector2d(0.1, 0.0);
        }
    }

    // npfg params
    npfg_.setAirspeedMax(airspeed_max); // XXX: consider augmenting these for any demanded fpa usage
    npfg_.setAirspeedNom(airspeed_nom);

    // initialize
    Eigen::Vector2d pos_k = pos_0;
    Eigen::Vector2d ground_vel_k = ground_vel_0;
    double ground_speed_k = ground_vel_k.norm();;
    Eigen::Vector2d unit_air_vel(cosf(heading_0), sinf(heading_0));
    double airspeed_k = airspeed_0;
    double wind_speed = wind_vel.norm();
    double dt_run_over = 0.0;
    double path_curvature;

    // allocate
    Eigen::Vector2d unit_path_tangent;
    Eigen::Vector2d closest_pt_on_path;

    int k = 0;

    // TODO: ENCAPSULATE

    // START SAMPLE TO PATH -------------------------------------------------------------------- START SAMPLE TO PATH //

    if (tg_mode_ == TGMode::GUIDE_TO_PATH_FROM_CURRENT_HEADING) { // ------------------------------------------------ //
        // follow guidance and propagate point mass dynamics until we reach the path

        if (path_sp.type == PathTypes::LOITER) {
            calcRefPoseOnLoiter2D(closest_pt_on_path, unit_path_tangent, path_sp.pos.segment(0,2), path_sp.signed_radius, pos_k, ground_vel_k, ground_speed_k);
            path_curvature = 1.0 / path_sp.signed_radius;
        }
        else if (path_sp.type == PathTypes::LINE) {
            unit_path_tangent = path_sp.dir.segment(0,2).normalized();
            path_curvature = 0.0;
        }
        else {
            // XXX: handle this
        }
        npfg_.setPathCurvature(path_curvature);

        bool converged = false;
        while (!converged && k < len_traj) {

            ground_speed_k = ground_vel_k.norm();

            if (path_sp.type == PathTypes::LOITER) {
                calcRefPoseOnLoiter2D(closest_pt_on_path, unit_path_tangent, path_sp.pos.segment(0,2), path_sp.signed_radius, pos_k, ground_vel_k, ground_speed_k);
            }
            else if (path_sp.type == PathTypes::LINE) {
                calcRefPoseOnLine2D(closest_pt_on_path, path_sp.pos.segment(0,2), unit_path_tangent, pos_k);
            }
            else {
                // XXX: handle this
            }
            Eigen::Vector2d track_error_vec = closest_pt_on_path - pos_k;

            const double track_error = track_error_vec.norm();
            Eigen::Vector2d unit_track_error;
            if (track_error < track_err_thres_) {
                unit_track_error = track_error_vec;
            }
            else {
                unit_track_error = track_error_vec / track_error;
            }

            const double track_error_bound = npfg_.calcTrackErrorBound(ground_speed_k);

            const double normalized_track_error = constrain(fabs(track_error / track_error_bound), 0.0, 1.0);

            const double look_ahead_angle = npfg_.calcLookAheadAngle(normalized_track_error);

            double track_proximity, inv_track_proximity;
            Eigen::Vector2d bearing_vec = npfg_.calcBearingVec(track_proximity, inv_track_proximity, unit_track_error, unit_path_tangent, look_ahead_angle);

            // *feedback airspeed for gain adjustment
            const double p_gain_adj = npfg_.adjustPGain(wind_speed / airspeed_k, inv_track_proximity);

            double wind_cross_bearing, wind_dot_bearing;
            npfg_.trigWindToBearing(wind_cross_bearing, wind_dot_bearing, wind_vel, bearing_vec);

            // adjust along-bearing minimum ground speed for constant set point and/or track error
            double min_ground_speed = npfg_.calcMinGroundSpeed(normalized_track_error);

            // get the heading reference assuming any airspeed incrementing is instantaneously achieved
            Eigen::Vector2d air_vel_ref = npfg_.calcRefAirVelocityFF(wind_vel, bearing_vec, wind_cross_bearing, wind_dot_bearing, wind_speed, min_ground_speed, false);
            airspeed_k = air_vel_ref.norm(); // instantaneous airspeed tracking

            const double feas = npfg_.calcBearingFeas(wind_cross_bearing, wind_dot_bearing, airspeed_k, wind_speed);

            // air velocity curvature adjustment -- feedback
            Eigen::Vector2d air_vel_ref_curv = air_vel_ref;
            npfg_.adjustRefAirVelForCurvature(air_vel_ref_curv, wind_vel, unit_path_tangent, wind_speed, airspeed_k, feas, track_proximity, p_gain_adj, false);

            // acceleration *feedback
            const double lateral_accel = npfg_.calcLateralAccel(airspeed_k * unit_air_vel, air_vel_ref_curv, airspeed_k, p_gain_adj);

            // output
            pos_traj.block(0,k,2,1) = pos_k;
            airsp_traj(0, k) = airspeed_k; // instantaneous airspeed tracking
            heading_traj(0, k) = atan2f(air_vel_ref(1), air_vel_ref(0)); // setting heading *reference
            roll_traj(0, k) = (en_off_track_roll_ff_) ? constrain(atanf(lateral_accel * INV_ONE_G), -roll_lim_rad, roll_lim_rad) : 0.0; // feed-forward guidance roll commands

            // propagate
            const double inv_airspeed_k = 1.0 / airspeed_k;
            ground_vel_k = unit_air_vel * airspeed_k + wind_vel;
            pos_k += dt_ * ground_vel_k;
            const double mult = dt_ * lateral_accel * inv_airspeed_k;
            unit_air_vel(0) += mult * -unit_air_vel(1);
            unit_air_vel(1) += mult * unit_air_vel(0);
            unit_air_vel.normalize();

            // check if converged
            const double dot_heading_err = unit_air_vel(0) * air_vel_ref(0) + unit_air_vel(1) * air_vel_ref(1);
            const double cross_heading_err = (unit_air_vel(0) * air_vel_ref(1) - unit_air_vel(1) * air_vel_ref(0))* inv_airspeed_k;
            bool dir_cond = dot_heading_err > 0.0 && fabs(cross_heading_err) < cross_err_thres_; // directional condition
            bool pos_cond = track_error < track_err_thres_; // positional condition
            converged = dir_cond && pos_cond;

            k++;
        }

        // point on next path (snap propagated position to path)
        if (path_sp.type == PathTypes::LOITER) {
            calcRefPoseOnLoiter2D(closest_pt_on_path, unit_path_tangent, path_sp.pos.segment(0,2), path_sp.signed_radius, pos_k, ground_vel_k, ground_speed_k);
        }
        else if (path_sp.type == PathTypes::LINE) {
            calcRefPoseOnLine2D(closest_pt_on_path, path_sp.pos.segment(0,2), unit_path_tangent, pos_k);
        }
        pos_k = closest_pt_on_path;

        dt_run_over = 0.0;
    }
    else { // default: tg_mode_ == TGMode::GUIDE_TO_PATH  // -------------------------------------------------------- //
        // follow guidance vector field to path

        if (path_sp.type == PathTypes::LOITER) {
            calcRefPoseOnLoiter2D(closest_pt_on_path, unit_path_tangent, path_sp.pos.segment(0,2), path_sp.signed_radius, pos_k, ground_vel_k, ground_speed_k);
            path_curvature = 1.0 / path_sp.signed_radius;
        }
        else if (path_sp.type == PathTypes::LINE) {
            unit_path_tangent = path_sp.dir.segment(0,2).normalized();
            path_curvature = 0.0;
        }
        else {
            // XXX: handle this
        }
        npfg_.setPathCurvature(path_curvature);

        bool converged = false;
        while (!converged && k < len_traj) {

            ground_speed_k = ground_vel_k.norm();

            if (path_sp.type == PathTypes::LOITER) {
                calcRefPoseOnLoiter2D(closest_pt_on_path, unit_path_tangent, path_sp.pos.segment(0,2), path_sp.signed_radius, pos_k, ground_vel_k, ground_speed_k);
            }
            else if (path_sp.type == PathTypes::LINE) {
                calcRefPoseOnLine2D(closest_pt_on_path, path_sp.pos.segment(0,2), unit_path_tangent, pos_k);
            }
            else {
                // XXX: handle this
            }
            Eigen::Vector2d track_error_vec = closest_pt_on_path - pos_k;

            const double track_error = track_error_vec.norm();
            Eigen::Vector2d unit_track_error;
            if (track_error < track_err_thres_) {
                unit_track_error = track_error_vec;
            }
            else {
                unit_track_error = track_error_vec / track_error;
            }

            const double track_error_bound = npfg_.calcTrackErrorBound(ground_speed_k);

            const double normalized_track_error = constrain(fabs(track_error / track_error_bound), 0.0, 1.0);

            const double look_ahead_angle = npfg_.calcLookAheadAngle(normalized_track_error);

            double track_proximity, inv_track_proximity;
            Eigen::Vector2d bearing_vec = npfg_.calcBearingVec(track_proximity, inv_track_proximity, unit_track_error, unit_path_tangent, look_ahead_angle);

//            // *feedback airspeed for gain adjustment
//            const double p_gain_adj = adjustPGain(wind_speed / airspeed_k, inv_track_proximity);

            double wind_cross_bearing, wind_dot_bearing;
            npfg_.trigWindToBearing(wind_cross_bearing, wind_dot_bearing, wind_vel, bearing_vec);

//                const double feas = calcBearingFeas(wind_cross_bearing, wind_dot_bearing, airspeed_k, wind_speed);

            // adjust along-bearing minimum ground speed for constant set point and/or track error
            double min_ground_speed = npfg_.calcMinGroundSpeed(normalized_track_error);

            // get the heading reference assuming any airspeed incrementing is instantaneously achieved
            Eigen::Vector2d air_vel_ref = npfg_.calcRefAirVelocityFF(wind_vel, bearing_vec, wind_cross_bearing, wind_dot_bearing, wind_speed, min_ground_speed, false);
            airspeed_k = air_vel_ref.norm(); // instantaneous airspeed tracking

//            // air velocity curvature adjustment -- feedback
//            Eigen::Vector2d air_vel_ref_curv = air_vel_ref;
//            adjustRefAirVelForCurvature(air_vel_ref_curv, wind_vel, unit_path_tangent, wind_speed, airspeed_k, feas, track_proximity, p_gain_adj, false);
//
//            // acceleration *feedback
//            const double lateral_accel = calcLateralAccel(airspeed_k * unit_air_vel, air_vel_ref_curv, airspeed_k, p_gain_adj);

            // output
            pos_traj.block(0,k,2,1) = pos_k;
            airsp_traj(0, k) = airspeed_k; // instantaneous airspeed tracking
            heading_traj(0, k) = atan2f(air_vel_ref(1), air_vel_ref(0)); // setting heading *reference
            roll_traj(0, k) = 0.0; // constrain(atanf(lateral_accel * INV_ONE_G), -roll_lim_rad, roll_lim_rad); // feed-forward guidance roll commands

            // propagate
            const double inv_airspeed_k = 1.0 / airspeed_k;
            ground_vel_k = air_vel_ref + wind_vel;
            pos_k += dt_ * ground_vel_k;

            // check if converged
//            const double dot_heading_err = unit_air_vel(0) * air_vel_ref(0) + unit_air_vel(1) * air_vel_ref(1);
//            const double cross_heading_err = (unit_air_vel(0) * air_vel_ref(1) - unit_air_vel(1) * air_vel_ref(0))* inv_airspeed_k;
//            bool dir_cond = dot_heading_err > 0.0 && fabs(cross_heading_err) < cross_err_thres_; // directional condition
            bool pos_cond = track_error < track_err_thres_; // positional condition
            converged = pos_cond; // dir_cond && pos_cond;

            k++;
        }

        // point on next path (snap propagated position to path)
        if (path_sp.type == PathTypes::LOITER) {
            calcRefPoseOnLoiter2D(closest_pt_on_path, unit_path_tangent, path_sp.pos.segment(0,2), path_sp.signed_radius, pos_k, ground_vel_k, ground_speed_k);
        }
        else if (path_sp.type == PathTypes::LINE) {
            calcRefPoseOnLine2D(closest_pt_on_path, path_sp.pos.segment(0,2), unit_path_tangent, pos_k);
        }
        pos_k = closest_pt_on_path;

        dt_run_over = 0.0;
    } // end sample to path

    // END SAMPLE TO PATH ------------------------------------------------------------------------ END SAMPLE TO PATH //

    // START SAMPLE PATH -------------------------------------------------------------------------- START SAMPLE PATH //

    // sample the path
    if (k < len_traj) {

        if (path_sp.type == PathTypes::LOITER) {

            // initial path step

            path_curvature = 1.0 / path_sp.signed_radius;
            npfg_.setPathCurvature(path_curvature);

            const double min_ground_speed = npfg_.calcMinGroundSpeed(0.0);

            double wind_cross_bearing, wind_dot_bearing;
            npfg_.trigWindToBearing(wind_cross_bearing, wind_dot_bearing, wind_vel, unit_path_tangent);

            Eigen::Vector2d air_vel_ref = npfg_.calcRefAirVelocityFF(wind_vel, unit_path_tangent, wind_cross_bearing, wind_dot_bearing, wind_speed, min_ground_speed, false);
            airspeed_k = air_vel_ref.norm();

            ground_vel_k = air_vel_ref + wind_vel;
            ground_speed_k = ground_vel_k.norm();

            // step through remainder of time step on path
            double mult = dt_run_over * ground_speed_k * path_curvature;
            unit_path_tangent(0) += mult * -unit_path_tangent(1);
            unit_path_tangent(1) += mult * unit_path_tangent(0);
            unit_path_tangent.normalize();

            // sample the path
            while (k < len_traj)  {

                const double min_ground_speed = npfg_.calcMinGroundSpeed(0.0);

                double wind_cross_bearing, wind_dot_bearing;
                npfg_.trigWindToBearing(wind_cross_bearing, wind_dot_bearing, wind_vel, unit_path_tangent.segment(0,2));

                // check feasibility
                if (wind_speed > airspeed_max && !npfg_.bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_max, wind_speed)) {
                    // only exit if the wind is both greater** than the max airspeed and tracking the path is infeasible
                    // i.e. we want to still be able to come to a complete stop on the path (which would be designated as an infeasible bearing)
                    break;
                }

                Eigen::Vector2d air_vel_ref = npfg_.calcRefAirVelocityFF(wind_vel, unit_path_tangent.segment(0,2), wind_cross_bearing, wind_dot_bearing, wind_speed, min_ground_speed, false);
                airspeed_k = air_vel_ref.norm();

                ground_vel_k = air_vel_ref + wind_vel;
                ground_speed_k = ground_vel_k.norm();

                const double feas = npfg_.calcBearingFeas(wind_cross_bearing, wind_dot_bearing, airspeed_k, wind_speed);
                const double p_gain_adj = npfg_.adjustPGain(wind_speed / airspeed_k, 0.0);

                Eigen::Vector2d air_vel_ref_curv = air_vel_ref;
                npfg_.adjustRefAirVelForCurvature(air_vel_ref_curv, wind_vel, unit_path_tangent.segment(0,2), wind_speed, airspeed_k, feas, 1.0, p_gain_adj, false);

                const double lateral_accel = npfg_.calcLateralAccel(air_vel_ref, air_vel_ref_curv, airspeed_k, p_gain_adj);

                // output
                pos_traj(0,k) = path_sp.pos(0) + unit_path_tangent(1) * path_sp.signed_radius;
                pos_traj(1,k) = path_sp.pos(1) + -unit_path_tangent(0) * path_sp.signed_radius;
                airsp_traj(0,k) = airspeed_k; // assume instantaneous airspeed tracking
                heading_traj(0,k) = atan2f(air_vel_ref(1), air_vel_ref(0));
                roll_traj(0,k) = constrain(atanf(lateral_accel / ONE_G), -roll_lim_rad, roll_lim_rad);

                // propagate
                double mult = dt_ * ground_speed_k * path_curvature;
                unit_path_tangent(0) += mult * -unit_path_tangent(1);
                unit_path_tangent(1) += mult * unit_path_tangent(0);
                unit_path_tangent.normalize();

                k++;
            }
        }
        else if (path_sp.type == PathTypes::LINE) {

            // initial path step

            path_curvature = 0.0;
            npfg_.setPathCurvature(path_curvature);

            const double min_ground_speed = npfg_.calcMinGroundSpeed(0.0);

            double wind_cross_bearing, wind_dot_bearing;
            npfg_.trigWindToBearing(wind_cross_bearing, wind_dot_bearing, wind_vel, unit_path_tangent);

            // check feasibility
            if (wind_speed <= airspeed_max || npfg_.bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_max, wind_speed)) {

                Eigen::Vector2d air_vel_ref = npfg_.calcRefAirVelocityFF(wind_vel, unit_path_tangent, wind_cross_bearing, wind_dot_bearing, wind_speed, min_ground_speed, false);
                airspeed_k = air_vel_ref.norm();
                const double heading_k = atan2f(air_vel_ref(1), air_vel_ref(0));

                ground_vel_k = air_vel_ref + wind_vel;
                ground_speed_k = ground_vel_k.norm();

                pos_k = closest_pt_on_path + dt_run_over * ground_vel_k;



                while (k < len_traj) {
                    // evaluate on track

                    // output
                    pos_traj.block(0,k,2,1) = pos_k;
                    airsp_traj(0, k) = airspeed_k; // assume instantaneous airspeed tracking
                    heading_traj(0, k) = heading_k;
                    roll_traj(0, k) = 0.0;

                    // propagate
                    pos_k += dt_ * ground_vel_k;

                    k++;
                }
            }
        } // end path type
    } // end sample path

    // END SAMPLE PATH ------------------------------------------------------------------------------ END SAMPLE PATH //

    // START DIVERGE FROM PATH -------------------------------------------------------------- START DIVERGE FROM PATH //

    // TODO: this is all copy/pasted.. encapsulate, add a bool for divergence, and use the other code snippets

    if (k < len_traj) {
        // this means we diverged from the track and need to follow infeasible guidance logic from here

        if (tg_mode_ == TGMode::GUIDE_TO_PATH_FROM_CURRENT_HEADING) { // ------------------------------------------- //
            // follow guidance and propagate point mass dynamics

            if (path_sp.type == PathTypes::LOITER) {
                calcRefPoseOnLoiter2D(closest_pt_on_path, unit_path_tangent, path_sp.pos.segment(0,2), path_sp.signed_radius, pos_k, ground_vel_k, ground_speed_k);
                path_curvature = 1.0 / path_sp.signed_radius;
            }
            else if (path_sp.type == PathTypes::LINE) {
                unit_path_tangent = path_sp.dir.segment(0,2).normalized();
                path_curvature = 0.0;
            }
            else {
                // XXX: handle this
            }
            npfg_.setPathCurvature(path_curvature);

            while (k < len_traj) {

                ground_speed_k = ground_vel_k.norm();

                if (path_sp.type == PathTypes::LOITER) {
                    calcRefPoseOnLoiter2D(closest_pt_on_path, unit_path_tangent, path_sp.pos.segment(0,2), path_sp.signed_radius, pos_k, ground_vel_k, ground_speed_k);
                }
                else if (path_sp.type == PathTypes::LINE) {
                    calcRefPoseOnLine2D(closest_pt_on_path, path_sp.pos.segment(0,2), unit_path_tangent, pos_k);
                }
                else {
                    // XXX: handle this
                }
                Eigen::Vector2d track_error_vec = closest_pt_on_path - pos_k;

                const double track_error = track_error_vec.norm();
                Eigen::Vector2d unit_track_error;
                if (track_error < track_err_thres_) {
                    unit_track_error = track_error_vec;
                }
                else {
                    unit_track_error = track_error_vec / track_error;
                }

                const double track_error_bound = npfg_.calcTrackErrorBound(ground_speed_k);

                const double normalized_track_error = constrain(fabs(track_error / track_error_bound), 0.0, 1.0);

                const double look_ahead_angle = npfg_.calcLookAheadAngle(normalized_track_error);

                double track_proximity, inv_track_proximity;
                Eigen::Vector2d bearing_vec = npfg_.calcBearingVec(track_proximity, inv_track_proximity, unit_track_error, unit_path_tangent, look_ahead_angle);

                // *feedback airspeed for gain adjustment
                const double p_gain_adj = npfg_.adjustPGain(wind_speed / airspeed_k, inv_track_proximity);

                double wind_cross_bearing, wind_dot_bearing;
                npfg_.trigWindToBearing(wind_cross_bearing, wind_dot_bearing, wind_vel, bearing_vec);

                // adjust along-bearing minimum ground speed for constant set point and/or track error
                double min_ground_speed = npfg_.calcMinGroundSpeed(normalized_track_error);

                // get the heading reference assuming any airspeed incrementing is instantaneously achieved
                Eigen::Vector2d air_vel_ref = npfg_.calcRefAirVelocityFF(wind_vel, bearing_vec, wind_cross_bearing, wind_dot_bearing, wind_speed, min_ground_speed, false);
                airspeed_k = air_vel_ref.norm(); // instantaneous airspeed tracking

                const double feas = npfg_.calcBearingFeas(wind_cross_bearing, wind_dot_bearing, airspeed_k, wind_speed);

                // air velocity curvature adjustment -- feedback
                Eigen::Vector2d air_vel_ref_curv = air_vel_ref;
                npfg_.adjustRefAirVelForCurvature(air_vel_ref_curv, wind_vel, unit_path_tangent, wind_speed, airspeed_k, feas, track_proximity, p_gain_adj, false);

                // acceleration *feedback
                const double lateral_accel = npfg_.calcLateralAccel(airspeed_k * unit_air_vel, air_vel_ref_curv, airspeed_k, p_gain_adj);

                // output
                pos_traj.block(0,k,2,1) = pos_k;
                airsp_traj(0, k) = airspeed_k; // instantaneous airspeed tracking
                heading_traj(0, k) = atan2f(air_vel_ref(1), air_vel_ref(0)); // setting heading *reference
                roll_traj(0, k) = (en_off_track_roll_ff_) ? constrain(atanf(lateral_accel * INV_ONE_G), -roll_lim_rad, roll_lim_rad) : 0.0; // feed-forward guidance roll commands

                // propagate
                const double inv_airspeed_k = 1.0 / airspeed_k;
                ground_vel_k = unit_air_vel * airspeed_k + wind_vel;
                pos_k += dt_ * ground_vel_k;
                const double mult = dt_ * lateral_accel * inv_airspeed_k;
                unit_air_vel(0) += mult * -unit_air_vel(1);
                unit_air_vel(1) += mult * unit_air_vel(0);
                unit_air_vel.normalize();

                k++;
            }
        }
        else { // default: tg_mode_ == TGMode::GUIDE_TO_PATH  // ---------------------------------------------------- //
            // follow guidance vector field to path

            if (path_sp.type == PathTypes::LOITER) {
                calcRefPoseOnLoiter2D(closest_pt_on_path, unit_path_tangent, path_sp.pos.segment(0,2), path_sp.signed_radius, pos_k, ground_vel_k, ground_speed_k);
                path_curvature = 1.0 / path_sp.signed_radius;
            }
            else if (path_sp.type == PathTypes::LINE) {
                unit_path_tangent = path_sp.dir.segment(0,2).normalized();
                path_curvature = 0.0;
            }
            else {
                // XXX: handle this
            }
            npfg_.setPathCurvature(path_curvature);

            while (k < len_traj) {

                ground_speed_k = ground_vel_k.norm();

                if (path_sp.type == PathTypes::LOITER) {
                    calcRefPoseOnLoiter2D(closest_pt_on_path, unit_path_tangent, path_sp.pos.segment(0,2), path_sp.signed_radius, pos_k, ground_vel_k, ground_speed_k);
                }
                else if (path_sp.type == PathTypes::LINE) {
                    calcRefPoseOnLine2D(closest_pt_on_path, path_sp.pos.segment(0,2), unit_path_tangent, pos_k);
                }
                else {
                    // XXX: handle this
                }
                Eigen::Vector2d track_error_vec = closest_pt_on_path - pos_k;

                const double track_error = track_error_vec.norm();
                Eigen::Vector2d unit_track_error;
                if (track_error < track_err_thres_) {
                    unit_track_error = track_error_vec;
                }
                else {
                    unit_track_error = track_error_vec / track_error;
                }

                const double track_error_bound = npfg_.calcTrackErrorBound(ground_speed_k);

                const double normalized_track_error = constrain(fabs(track_error / track_error_bound), 0.0, 1.0);

                const double look_ahead_angle = npfg_.calcLookAheadAngle(normalized_track_error);

                double track_proximity, inv_track_proximity;
                Eigen::Vector2d bearing_vec = npfg_.calcBearingVec(track_proximity, inv_track_proximity, unit_track_error, unit_path_tangent, look_ahead_angle);

//                // *feedback airspeed for gain adjustment
//                const double p_gain_adj = adjustPGain(wind_speed / airspeed_k, inv_track_proximity);

                double wind_cross_bearing, wind_dot_bearing;
                npfg_.trigWindToBearing(wind_cross_bearing, wind_dot_bearing, wind_vel, bearing_vec);

//                    const double feas = calcBearingFeas(wind_cross_bearing, wind_dot_bearing, airspeed_k, wind_speed);

                // adjust along-bearing minimum ground speed for constant set point and/or track error
                double min_ground_speed = npfg_.calcMinGroundSpeed(normalized_track_error);

                // get the heading reference assuming any airspeed incrementing is instantaneously achieved
                Eigen::Vector2d air_vel_ref = npfg_.calcRefAirVelocityFF(wind_vel, bearing_vec, wind_cross_bearing, wind_dot_bearing, wind_speed, min_ground_speed, false);
                airspeed_k = air_vel_ref.norm(); // instantaneous airspeed tracking

//                // air velocity curvature adjustment -- feedback
//                Eigen::Vector2d air_vel_ref_curv = air_vel_ref;
//                adjustRefAirVelForCurvature(air_vel_ref_curv, wind_vel, unit_path_tangent, wind_speed, airspeed, feas, track_proximity, p_gain_adj, false);
//
//                // acceleration *feedback
//                const double lateral_accel = calcLateralAccel(airspeed_k * unit_air_vel, air_vel_ref_curv, airspeed_k, p_gain_adj);

                // output
                pos_traj.block(0,k,2,1) = pos_k;
                airsp_traj(0, k) = airspeed_k; // instantaneous airspeed tracking
                heading_traj(0, k) = atan2f(air_vel_ref(1), air_vel_ref(0)); // setting heading *reference
                roll_traj(0, k) = 0.0; // constrain(atanf(lateral_accel * INV_ONE_G), -roll_lim_rad, roll_lim_rad); // feed-forward guidance roll commands

                // propagate
                const double inv_airspeed_k = 1.0 / airspeed_k;
                ground_vel_k = air_vel_ref + wind_vel;
                pos_k += dt_ * ground_vel_k;

                k++;
            }
        }
    }

    // END DIVERGE FROM PATH ------------------------------------------------------------------ END DIVERGE FROM PATH //

} // genTrajectoryToPath2D

double TrajectoryGenerator::evaluateLongitudinalGuidance(double &err_lon, const PathSetpoint &path_sp, const Eigen::Vector3d &veh_pos,
    const double ground_speed, const double airspeed, const double wind_vel_d)
{
    double pos_d_ref;
    if (path_sp.type == PathTypes::LOITER) {
        pos_d_ref = path_sp.pos(2);
    }
    else if (path_sp.type == PathTypes::LINE) {
        Eigen::Vector3d closest_pt_on_path;
        calcRefPoseOnLine(closest_pt_on_path, path_sp.pos, path_sp.dir, veh_pos);
        pos_d_ref = closest_pt_on_path(2);
    }
    else {
        // XXX: handle this
    }

    return pwqg_.evaluate(err_lon, pos_d_ref, path_sp.dir(2), veh_pos(2), ground_speed, airspeed, wind_vel_d); // return fpa
} // evaluateLongitudinalGuidance

/*
    END TRAJECTORY GENERATION
*/

/*
    MANUAL CONTROL
*/

double TrajectoryGenerator::getManualFPASetpoint(double &err_lon, const ManualControlSetpoint &manual_control_sp,
    const double terr_alt, const double veh_pos_d, const double ground_speed, const double airspeed, const double wind_vel_d)
{
    // NOTE: manual setpoints should ALREADY BE SET IN STRUCT after calling the updateManualSetpoint() function BEFORE this one

    if (manual_control_sp.lon_mode == MCLongitudinalModes::FPA) {
        // track the commanded flight path angle

        err_lon = 0.0;

        return manual_control_sp.fpa;
    }
    else if (manual_control_sp.lon_mode == MCLongitudinalModes::ALTITUDE) {
        // track manual bearing and altitude setpoints

        return pwqg_.evaluate(err_lon, -manual_control_sp.alt, 0.0, veh_pos_d, ground_speed, airspeed, wind_vel_d);
    }
    else if (manual_control_sp.lon_mode == MCLongitudinalModes::TERRAIN_ALTITUDE) {
        // track manual bearing and relative altitude setpoints

        // set absolute altitude based on node-wise terrain height
        double alt = terr_alt + manual_control_sp.rel_alt;

        // terrain slope
        // TODO: for now flat.. could potentially calculate the slope based on the current course .. it is calculated anyway internally in the cost jacobian.
        const double unit_path_tangent_d = 0.0;

        return pwqg_.evaluate(err_lon, -alt, unit_path_tangent_d, veh_pos_d, ground_speed, airspeed, wind_vel_d);
    }

    return 0.0; // should not reach here..
} // getFPASetpoint

void TrajectoryGenerator::updateManualSetpoint(ManualControlSetpoint &setpoint, const ManualControlInput &input,
    const double veh_pos_d, const double heading, const double terr_alt, const double airsp_min, const double airsp_max,
    const double roll_lim)
{
    if (setpoint.enabled) {

        // check mode
        mc_lon_mode_changed_ = setpoint.lon_mode != mc_last_lon_mode_;

        if (mc_was_disabled_ || mc_lon_mode_changed_) {
            // reset / initialize

            // set current altitude
            alt_hold_ = -veh_pos_d;
            setpoint.alt = alt_hold_;

            if (setpoint.lon_mode == MCLongitudinalModes::TERRAIN_ALTITUDE) {
                // set relative altitude
                setpoint.rel_alt = std::max(setpoint.alt - terr_alt, 0.0); // must be above ground

                // set absolute altitude
                alt_hold_ = terr_alt + setpoint.rel_alt;
                setpoint.alt = alt_hold_;
            }

            mc_was_disabled_ = false;
        }

        mc_lat_mode_changed_ = setpoint.lat_mode != mc_last_lat_mode_;

        if (mc_was_disabled_ || mc_lat_mode_changed_) {
            // reset / initialize

            setpoint.roll = 0.0;

            heading_hold_ = heading;
            setpoint.heading = heading_hold_;

            mc_was_disabled_ = false;
        }

        // roll stick inputs
        double y_input = 0.0;
        bool hold_it = false;
        if (input.y < -MANUAL_CONTROL_DZ) {
            y_input = constrain((input.y + MANUAL_CONTROL_DZ) * INV_ONE_MINUS_DZ, -1.0, 0.0);
        }
        else if (input.y > MANUAL_CONTROL_DZ) {
            y_input = constrain((input.y - MANUAL_CONTROL_DZ) * INV_ONE_MINUS_DZ, 0.0, 1.0);
        }
        else {
            // in the dead-zone, maintain the hold
            hold_it = true;
        }

        // what to do with roll stick inputs
        if (setpoint.lat_mode == MCLateralModes::HEADING) {
            // roll stick controls heading delta

            if (hold_it) {
                setpoint.heading = heading_hold_;
            }
            else {
                const double heading_error = wrapPi(heading - heading_hold_);

                const double delta = y_input * max_delta_heading_;

                setpoint.heading = wrapPi(heading_hold_ + delta);

                // update the heading hold by any progress we've made in the *correct direction, but not exceeding the new setpoint
                if (delta < 0.0) {
                    heading_hold_ = wrapPi(heading_hold_ + constrain(heading_error, delta, 0.0));
                }
                else {
                    heading_hold_ = wrapPi(heading_hold_ + constrain(heading_error, 0.0, delta));
                }
            }

            setpoint.roll = 0.0;
        }
        else { // default == MCLateralModes::ROLL
            // roll stick controls (setpoint of) roll setpoint

            setpoint.roll = y_input * roll_lim;
        }

        // pitch stick inputs
        double x_input = 0.0;
        hold_it = false;
        if (input.x < -MANUAL_CONTROL_DZ) {
            x_input = -constrain((input.x + MANUAL_CONTROL_DZ) * INV_ONE_MINUS_DZ, -1.0, 0.0);
        }
        else if (input.x > MANUAL_CONTROL_DZ) {
            x_input = -constrain((input.x - MANUAL_CONTROL_DZ) * INV_ONE_MINUS_DZ, 0.0, 1.0); //TODO: again should have separate sink and climb params
        }
        else {
            // in the dead-zone, maintain the hold
            hold_it = true;
        }

        // what to do with pitch stick inputs
        if (setpoint.lon_mode == MCLongitudinalModes::TERRAIN_ALTITUDE) {

            if (hold_it) {
                setpoint.rel_alt = alt_hold_;
            }
            else {

                const double rel_alt_error = (-veh_pos_d - terr_alt) - alt_hold_;

                const double delta = x_input * max_delta_alt_;

                setpoint.rel_alt = std::max(alt_hold_ + delta, 0.0);

                // update the relative altitude hold by any progress we've made in the *correct direction, but not exceeding the new setpoint
                if (delta < 0.0) {
                    alt_hold_ = std::max(alt_hold_ + constrain(rel_alt_error, delta, 0.0), 0.0);
                }
                else {
                    alt_hold_ = std::max(alt_hold_ + constrain(rel_alt_error, 0.0, delta), 0.0);
                }
            }
        }
        else if (setpoint.lon_mode == MCLongitudinalModes::ALTITUDE) {

            if (hold_it) {
                setpoint.alt = alt_hold_;
            }
            else {

                const double alt_error = -veh_pos_d - alt_hold_;

                const double delta = x_input * max_delta_alt_;

                setpoint.alt = std::max(alt_hold_ + delta, 0.0);

                // update the altitude hold by any progress we've made in the *correct direction, but not exceeding the new setpoint
                if (delta < 0.0) {
                    alt_hold_ = std::max(alt_hold_ + constrain(alt_error, delta, 0.0), 0.0);
                }
                else {
                    alt_hold_ = std::max(alt_hold_ + constrain(alt_error, 0.0, delta), 0.0);
                }
            }
        }
        else { // default == MCLongitudinalModes::FPA
            setpoint.fpa = x_input * pwqg_.getFPAAppMax();
        }

        // throttle stick controls airspeed set point
        setpoint.airspeed = (airsp_max - airsp_min) * constrain(input.z, 0.0, 1.0) + airsp_min;
    }
    else {
        mc_was_disabled_ = true;
    }

    mc_last_lat_mode_ = setpoint.lat_mode;
    mc_last_lon_mode_ = setpoint.lon_mode;
} // updateManualSetpoint

/*
    END MANUAL CONTROL
*/

} // namespace fw_nmpc
