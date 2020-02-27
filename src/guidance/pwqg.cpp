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

double PWQG::evaluate(double &err_lon, const double d_ref, const double unit_path_tangent_d, const double veh_pos_d,
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

        return:
        fpa_sp: flight path angle setpoint [rad]
    */

    // position error
    err_lon = d_ref - veh_pos_d;

    // track-error boundaries
    double err_b_lon;
    if (fix_vert_pos_err_bnd_) {
        err_b_lon = vert_pos_err_bnd_;
    }
    else {
        err_b_lon = calcTrackErrorBound(ground_speed); // longitudinal track-error boundary
    }

    // unit track error
    const double unit_err_lon = constrain(fabs(err_lon)/err_b_lon, 0.0, 1.0);

    // quadratic transition variables
    const double thetal_lon = -unit_err_lon*(unit_err_lon - 2.0);

    // sign of longitudinal position error
    const double sign_e_lon = (err_lon < 0.0) ? 1.0 : -1.0; // note this is flipped to "up" = positive

    // adjust for wind (small angle assumption)
    const double fpa_path = -(ground_speed * unit_path_tangent_d - wind_vel_d) / airspeed;

    // unit z velocity setpoint
    return constrain((1.0 - thetal_lon) * fpa_path + thetal_lon * fpa_app_max_ * sign_e_lon, -fpa_app_max_, fpa_app_max_);
} // evaluate

} // namespace fw_nmpc
