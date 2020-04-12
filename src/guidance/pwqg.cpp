#include <fw_nmpc/guidance/pwqg.h>
#include <fw_nmpc/common/helpers.h>

namespace fw_nmpc {

PWQG::PWQG() :
    fix_vert_pos_err_bnd_(true),
    time_const_(5.0),
    fpa_app_max_(0.34),
    vert_pos_err_bnd_(5.0)
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

double PWQG::evaluate(double &err_lon, double *jac_fpa_sp, const double d_ref, const double unit_path_tangent_d, const double veh_pos_d,
    const double ground_speed, const double airspeed, const double wind_vel_d)
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
            TODO...

        return:
        fpa_sp: flight path angle setpoint [rad]
    */

    // position error
    err_lon = d_ref - veh_pos_d;

    // track-error boundaries
    double inv_err_b_lon;
    if (fix_vert_pos_err_bnd_) {
        inv_err_b_lon = 1.0 / vert_pos_err_bnd_;
    }
    else {
        inv_err_b_lon = 1.0 / calcTrackErrorBound(ground_speed); // longitudinal track-error boundary
    }

    // on track FPA -- adjust for wind (small angle assumption)
    const double fpa_path = constrain(-(ground_speed * unit_path_tangent_d - wind_vel_d) / airspeed, -fpa_app_max_, fpa_app_max_);

    // unit track error shift
    const double sign_fpa_path = (fpa_path < 0.0) ? -1.0 : 1.0;
    const double delta_err_lon = sign_fpa_path * (sqrt(1.0 - sign_fpa_path * fpa_path / fpa_app_max_) - 1.0);

    // unit track error
    const double unit_err_lon = constrain(err_lon * inv_err_b_lon + delta_err_lon, -1.0, 1.0);

    // FPA setpoint
    if (unit_err_lon < 0.0) {
        jac_fpa_sp[0] = 2.0 * inv_err_b_lon * fpa_app_max_ * (1.0 + unit_err_lon);
        return -fpa_app_max_ * unit_err_lon * (unit_err_lon + 2.0);
    }
    else {
        jac_fpa_sp[0] = -2.0 * inv_err_b_lon * fpa_app_max_ * (unit_err_lon - 1.0);
        return fpa_app_max_ * unit_err_lon * (unit_err_lon - 2.0);
    }
} // evaluate

} // namespace fw_nmpc
