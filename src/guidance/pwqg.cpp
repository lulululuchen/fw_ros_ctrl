#include <fw_nmpc/guidance/pwqg.h>
#include <fw_nmpc/common/helpers.h>

namespace fw_nmpc {

PWQG::PWQG() :
    fix_vert_pos_err_bnd_(true),
    time_const_(5.0),
    fpa_climb_(0.34),
    fpa_sink_(-0.17),
    vert_pos_err_bnd_(5.0),
    pos_carrot_scale_(1.0),
    pos_carrot_dyn_(CarrotDynamics::LINEAR),
    max_unit_err_lon_(0.95)
{} // PWQG

double PWQG::calcTrackErrorBound(const double ground_speed)
{
    if (ground_speed > 1.0) {
      return ground_speed * time_const_;
    }
    else {
      // limit bound to some minimum ground speed to avoid singularities in track error normalization
      // the following equation assumes ground speed minimum = 1.0
      return 0.5 * time_const_ * (ground_speed * ground_speed + 1.0);
    }
} // calcTrackErrorBound

double PWQG::calcPositionCarrot(const double veh_pos_d, const double d_ref, const double ground_speed)
{
    // position error
    const double err_lon = d_ref - veh_pos_d;

    if (pos_carrot_dyn_ == CarrotDynamics::QUADRATIC) {

        // track-error boundaries
        double inv_err_b_lon, eb;
        if (fix_vert_pos_err_bnd_) {
            eb = vert_pos_err_bnd_;
            inv_err_b_lon = 1.0 / eb;
        }
        else {
            eb = calcTrackErrorBound(ground_speed);
            inv_err_b_lon = 1.0 / eb; // longitudinal track-error boundary
        }

        const double unit_err_lon = constrain(err_lon * inv_err_b_lon, -1.0, 1.0);

        // position carrot offset
        return unit_err_lon * eb * pos_carrot_scale_;
    }
    else { // pos_carrot_dyn_ == CarrotDynamics::LINEAR

        // position carrot offset
        if (fix_vert_pos_err_bnd_) {
            return constrain(err_lon, -vert_pos_err_bnd_, vert_pos_err_bnd_) * pos_carrot_scale_;
        }
        else {
            const double eb = calcTrackErrorBound(ground_speed);
            return constrain(err_lon, -eb, eb) * pos_carrot_scale_;
        }
    }
} // calcPositionCarrot

double PWQG::evaluate(double &err_lon, double *jac_fpa_sp, bool &feas, const double d_ref, const double unit_path_tangent_d, const double veh_pos_d,
    const double ground_speed, const double airspeed, const double wind_vel_d, const Eigen::Vector3d &ground_vel, const Eigen::Vector3d &air_vel)
{
    /*
        Piecewise Quadratic Longitudinal Path Following Guidance Logic

        inputs:
        d_ref (3,1): down reference [m]
        unit_path_tangent_d: unit down path tangent at d_ref [~]
        veh_pos_d: vehicle position, NED [m]
        ground_speed: vehicle total ground speed [m/s]
        airspeed: vehicle total airspeed [m/s]
        wind_vel_d: wind down velocity [m/s]

        outputs:
        err_lon: longitudinal (vertical) signed track-error [m]
        jac_fpa_sp: jacobian of the flight path angle setpoint // TODO: maybe add a disable for this if we don't care to calculate.. but it's pretty cheap anyway.
            w.r.t.:
            veh_pos_d
            airspeed
            TODO...
        feas: tracking feasibility

        return:
        fpa_sp: flight path angle setpoint [rad]
    */

    // position error
    err_lon = d_ref - veh_pos_d;

    const double fpa_ratio = fabs(fpa_sink_ / fpa_climb_);
    const double fpa_half_span = (fabs(fpa_sink_) + fabs(fpa_climb_)) * 0.5;
    const double fpa_avg = 0.5 * (fpa_sink_ + fpa_climb_);

    // track-error boundaries
    double inv_err_b_lon;
//    if (fix_vert_pos_err_bnd_) {
        inv_err_b_lon = 1.0 / (0.5 * vert_pos_err_bnd_ * (1.0 + fpa_ratio));
//    }
//    else {
//        inv_err_b_lon = 1.0 / calcTrackErrorBound(ground_speed); // longitudinal track-error boundary
//    }

    // on track FPA -- adjust for wind (small angle assumption)
    const double inv_airspeed = 1.0 / airspeed;
    const double fpa_path_uncnstr = -(ground_speed * unit_path_tangent_d - wind_vel_d) * inv_airspeed;
    const double fpa_path = constrain(fpa_path_uncnstr, fpa_sink_, fpa_climb_);
    double d_fpa_path_d_airsp;
    if (ground_speed < 1.0) {
        d_fpa_path_d_airsp = -ground_vel.dot(air_vel) * unit_path_tangent_d * inv_airspeed * inv_airspeed - fpa_path * inv_airspeed;
    }
    else {
        d_fpa_path_d_airsp = -ground_vel.dot(air_vel) / ground_speed * unit_path_tangent_d * inv_airspeed * inv_airspeed - fpa_path * inv_airspeed;
    }

    // tracking feasibility
    feas = fpa_sink_ < fpa_path_uncnstr && fpa_path_uncnstr < fpa_climb_;

    // unit track error shift
    const double unit_delta_fpa = (fpa_avg - fpa_path) / fpa_half_span;
    double unit_delta_err_lon, d_unit_delta_err_lon_d_fpa_path;
    if (unit_delta_fpa < 0.0) {
        double sqrt_term = sqrt(1.0 + unit_delta_fpa);
        unit_delta_err_lon = sqrt_term - 1.0;
        d_unit_delta_err_lon_d_fpa_path = (unit_delta_fpa > -1.0) ? -0.5 / (sqrt_term * fpa_half_span) : 0.0;
    }
    else {
        double sqrt_term = sqrt(1.0 - unit_delta_fpa);
        unit_delta_err_lon = 1.0 - sqrt_term;
        d_unit_delta_err_lon_d_fpa_path = (unit_delta_fpa < 1.0) ? -0.5 / (sqrt_term * fpa_half_span) : 0.0;
    }

    // unit track error
    const double unit_err_lon = constrain(err_lon * inv_err_b_lon + unit_delta_err_lon, -1.0, 1.0);

    // FPA setpoint
    if (unit_err_lon < 0.0) {
        const double d_fpa_sp_d_unit_err_lon = -2.0 * fpa_half_span * (1.0 + unit_err_lon);
        const double d_fpa_sp_d_unit_err_lon_max = -2.0 * fpa_half_span * (1.0 + -max_unit_err_lon_);
        jac_fpa_sp[0] = std::min(d_fpa_sp_d_unit_err_lon, d_fpa_sp_d_unit_err_lon_max) * -inv_err_b_lon; // w.r.t. pos_d
        jac_fpa_sp[1] = d_fpa_sp_d_unit_err_lon * d_unit_delta_err_lon_d_fpa_path * d_fpa_path_d_airsp; // w.r.t. airspeed
        return -unit_err_lon * (unit_err_lon + 2.0) * fpa_half_span + fpa_avg;
    }
    else {
        const double d_fpa_sp_d_unit_err_lon = 2.0 * fpa_half_span * (unit_err_lon - 1.0);
        const double d_fpa_sp_d_unit_err_lon_max = 2.0 * fpa_half_span * (max_unit_err_lon_ - 1.0);
        jac_fpa_sp[0] = std::min(d_fpa_sp_d_unit_err_lon, d_fpa_sp_d_unit_err_lon_max) * -inv_err_b_lon; // w.r.t. pos_d
        jac_fpa_sp[1] = d_fpa_sp_d_unit_err_lon * d_unit_delta_err_lon_d_fpa_path * d_fpa_path_d_airsp; // w.r.t. airspeed
        return unit_err_lon * (unit_err_lon - 2.0) * fpa_half_span + fpa_avg;
    }
} // evaluate

} // namespace fw_nmpc
