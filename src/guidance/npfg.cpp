#include <fw_nmpc/guidance/npfg.h>
#include <fw_nmpc/common/helpers.h>
#include <math.h>

namespace fw_nmpc {

NPFG::NPFG() :
    feas_(1.0),
    lateral_accel_(0.0),
    path_curvature_(0.0),
    track_error_bound_(1.0),
    bearing_vec_(Eigen::Vector2d(1.0, 0.0)),
    p_gain_(0.11),
    p_gain_adj_(0.11),
    time_const_(7.0),
    airspeed_nom_(14.0),
    airspeed_max_(20.0),
    airspeed_ref_(14.0),
    wind_ratio_buf_(0.1),
    inv_nte_fraction_(0.5),
    min_ground_speed_e_(0.0),
    min_ground_speed_e_max_(0.0),
    min_ground_speed_g_(0.0),
    en_wind_excess_regulation_(true),
    en_track_keeping_(false),
    en_min_ground_speed_(false),
    feed_forward_air_vel_(false),
    en_backwards_solution_(false),
    nom_heading_rate_(0.6)
{} // NPFG

double NPFG::calcTrackErrorBound(const double ground_speed)
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

double NPFG::calcLookAheadAngle(const double normalized_track_error)
{
    return M_PI_2 * (normalized_track_error - 1.0) * (normalized_track_error - 1.0);
} // calcLookAheadAngle

Eigen::Vector2d NPFG::calcBearingVec(double &track_proximity, double &inv_track_proximity,
    const Eigen::Vector2d &unit_track_error, const Eigen::Vector2d &unit_path_tangent, const double look_ahead_angle)
{
    /*
        calculate the bearing vector and track proximity smoother from the look-ahead angle mapping
    */

    const double cos_look_ahead_angle = cosf(look_ahead_angle);
	const double sin_look_ahead_angle = sinf(look_ahead_angle);

    track_proximity = sin_look_ahead_angle * sin_look_ahead_angle;
    inv_track_proximity = cos_look_ahead_angle * cos_look_ahead_angle;

    return cos_look_ahead_angle * unit_track_error + sin_look_ahead_angle * unit_path_tangent;
} // calcBearingVec

double NPFG::adjustPGain(const double wind_ratio, const double normalized_track_error)
{
    // take maximum of minimum gain for given curvature and wind ratio or operator defined gain
	double p_gain_max;
    if (wind_ratio > 1.0) {
        // excess wind case
        p_gain_max = std::max(P_GAIN_MIN_MULT * (1.0 + wind_ratio) * (1.0 + wind_ratio) * fabs(path_curvature_), p_gain_);
    }
    else {
        // lower wind case
        p_gain_max = std::max(P_GAIN_MIN_MULT * 4.0 * fabs(path_curvature_), p_gain_);
    }
	// linearly interpolate between operator defined gain and minimum necessary gain when within proximity to track
	return p_gain_max + normalized_track_error * (p_gain_ - p_gain_max);
} // adjustPGain

double NPFG::calcMinGroundSpeed(const double normalized_track_error)
{
    min_ground_speed_e_ = 0.0;
    if (en_track_keeping_ * en_wind_excess_regulation_) {
        min_ground_speed_e_ = min_ground_speed_e_max_ * constrain(normalized_track_error * inv_nte_fraction_, 0.0, 1.0);
    }

    double min_ground_speed_g = 0.0;
    if (en_min_ground_speed_ * en_wind_excess_regulation_) {
        min_ground_speed_g = min_ground_speed_g_;
    }

    return std::max(min_ground_speed_e_, min_ground_speed_g);
} // calcMinGroundSpeed

void NPFG::trigWindToBearing(double &wind_cross_bearing, double &wind_dot_bearing, const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec)
{
    wind_cross_bearing = wind_vel(0) * bearing_vec(1) - wind_vel(1) * bearing_vec(0);
	wind_dot_bearing = wind_vel(0) * bearing_vec(0) + wind_vel(1) * bearing_vec(1);
} // trigWindToBearing

double NPFG::projectAirspOnBearing(const double airspeed, const double wind_cross_bearing)
{
    return sqrt(std::max(airspeed * airspeed - wind_cross_bearing * wind_cross_bearing, 0.0));
} // projectAirspOnBearing

int NPFG::bearingIsFeasible(const double wind_cross_bearing, const double wind_dot_bearing, const double airspeed, const double wind_speed)
{
    return (fabs(wind_cross_bearing) < airspeed) && ((wind_dot_bearing > 0.0) || (wind_speed < airspeed));
} // bearingIsFeasible

double NPFG::calcBearingFeas(const double wind_cross_bearing, const double wind_dot_bearing, const double airspeed, const double wind_speed)
{
    double sin_lambda; // in [0, 1]
    if (wind_dot_bearing <= 0.0) {
        sin_lambda = 1.0;
    }
    else {
        sin_lambda = fabs(wind_cross_bearing / wind_speed);
    }

    const double wind_ratio = wind_speed / airspeed; // XXX: could potentially be (*is..) a duplicate calculation

	// upper and lower feasibility barriers
	double wind_ratio_ub, wind_ratio_lb;
	if (sin_lambda < LAMBDA_CO) { // small angle approx.
		// linear finite cut-off
		const double mx = M_CO * (LAMBDA_CO - sin_lambda);
		const double wind_ratio_ub_co = ONE_OVER_S_LAMBDA_CO;
		wind_ratio_ub = wind_ratio_ub_co + mx;
		const double wind_ratio_lb_co = (ONE_OVER_S_LAMBDA_CO - 2.0) * wind_ratio_buf_ + 1.0;
		wind_ratio_lb = wind_ratio_lb_co + wind_ratio_buf_ * mx;
	}
    else {
        const double one_over_s_lambda = 1.0 / sin_lambda;
        wind_ratio_ub = one_over_s_lambda;
        wind_ratio_lb = (one_over_s_lambda - 2.0) * wind_ratio_buf_ + 1.0;
    }

	// calculate bearing feasibility
	double feas;
    if (wind_ratio > wind_ratio_ub) {
        // infeasible
        feas = 0.0;
    }
    else if (wind_ratio > wind_ratio_lb) {
        // partially feasible
        // smoothly transition from fully feasible to infeasible
        feas = cosf(M_PI_2 * constrain((wind_ratio - wind_ratio_lb) / (wind_ratio_ub - wind_ratio_lb), 0.0, 1.0));
        feas *= feas;
	}
    else {
        // feasible
        feas = 1.0;
    }

    return feas;
} // calcBearingFeas

Eigen::Vector2d NPFG::calcRefAirVelocityFF(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec,
    const double wind_cross_bearing, const double wind_dot_bearing, const double wind_speed, const double min_ground_speed,
    bool use_backwards_solution)
{
    /*
        determine air velocity reference without curvature compensation
        :: with "optimal" airspeed reference compensation
    */

    Eigen::Vector2d air_vel_ref;

    if (min_ground_speed > wind_dot_bearing && (en_min_ground_speed_ || en_track_keeping_) * en_wind_excess_regulation_) {
        // minimum ground speed and/or track keeping

        // airspeed required to achieve min. ground speed along bearing vector
        const double airspeed_min = sqrt((min_ground_speed - wind_dot_bearing) * (min_ground_speed - wind_dot_bearing)
            + wind_cross_bearing * wind_cross_bearing);

        if (airspeed_min > airspeed_max_) {
            if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_max_, wind_speed)) {
                const double airsp_dot_bearing = projectAirspOnBearing(airspeed_max_, wind_cross_bearing);
                air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
            }
            else {
                air_vel_ref = calcInfeasibleAirVelRef(wind_vel, bearing_vec, wind_speed, airspeed_max_);
            }
        }
        else if (airspeed_min > airspeed_nom_) {
            const double airsp_dot_bearing = projectAirspOnBearing(airspeed_min, wind_cross_bearing);
            air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
        }
        else {
            const double airsp_dot_bearing = projectAirspOnBearing(airspeed_nom_, wind_cross_bearing);
            if (use_backwards_solution) {
                air_vel_ref = rotate2d(wind_cross_bearing, -airsp_dot_bearing, bearing_vec);
            }
            else {
                air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
            }
        }
    }
    else {
        // wind excess regulation and/or mitigation

        if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_nom_, wind_speed)) {
            const double airsp_dot_bearing = projectAirspOnBearing(airspeed_nom_, wind_cross_bearing);
            if (use_backwards_solution) {
                air_vel_ref = rotate2d(wind_cross_bearing, -airsp_dot_bearing, bearing_vec);
            }
            else {
                air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
            }
        }
        else if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_max_, wind_speed) && en_wind_excess_regulation_) {
            if (wind_dot_bearing <= 0.0) {
                air_vel_ref = wind_vel;
            }
            else {
                const double airsp_dot_bearing = 0.0; // right angle to the bearing line gives minimum airspeed usage, zero forward ground speed
                air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
            }
        }
        else {
            // infeasible
            const double airspeed_input = (en_wind_excess_regulation_) ? airspeed_max_ : airspeed_nom_;
            air_vel_ref = calcInfeasibleAirVelRef(wind_vel, bearing_vec, wind_speed, airspeed_input);
        }
    }

    return air_vel_ref;
} // calcRefAirVelocityFF

Eigen::Vector2d NPFG::calcRefAirVelocityFB(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec,
    const double wind_cross_bearing, const double wind_dot_bearing, const double wind_speed, const double airspeed,
    bool use_backwards_solution)
{
    /*
        determine air velocity reference without curvature compensation
        :: this is the feedback formulation (at current airspeed), no airspeed reference compensation
    */

    Eigen::Vector2d air_vel_ref;

    if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed, wind_speed)) {
        const double airsp_dot_bearing = projectAirspOnBearing(airspeed, wind_cross_bearing);
        if (use_backwards_solution) {
            air_vel_ref = rotate2d(wind_cross_bearing, -airsp_dot_bearing, bearing_vec);
        }
        else {
            air_vel_ref = rotate2d(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
        }
    }
    else {
        air_vel_ref = calcInfeasibleAirVelRef(wind_vel, bearing_vec, wind_speed, airspeed);
    }

    return air_vel_ref;
} // calcRefAirVelocityFB

Eigen::Vector2d NPFG::calcInfeasibleAirVelRef(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec, const double wind_speed, const double airspeed)
{
    Eigen::Vector2d air_vel_ref = sqrt(std::max(wind_speed * wind_speed - airspeed * airspeed, 0.0)) * bearing_vec - wind_vel;
    return air_vel_ref.normalized() * airspeed;
} // calcInfeasibleAirVelRef

double NPFG::calcRefAirspeed(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec,
    const double wind_cross_bearing, const double wind_dot_bearing, const double wind_speed, const double min_ground_speed)
{
    /*
        determine airspeed reference without curvature compensation
        :: with "optimal" airspeed reference compensation
    */

    double airspeed_ref;

    if (min_ground_speed > wind_dot_bearing && (en_min_ground_speed_ || en_track_keeping_) * en_wind_excess_regulation_) {
        // minimum ground speed and/or track keeping

        // airspeed required to achieve min. ground speed along bearing vector
        const double airspeed_min = sqrt((min_ground_speed - wind_dot_bearing) * (min_ground_speed - wind_dot_bearing)
            + wind_cross_bearing * wind_cross_bearing);

        airspeed_ref = constrain(airspeed_min, airspeed_nom_, airspeed_max_);
    }
    else {
        // wind excess regulation and/or mitigation

        if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_nom_, wind_speed)) {
            airspeed_ref = airspeed_nom_;
        }
        else if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_max_, wind_speed) && en_wind_excess_regulation_) {

            if (wind_dot_bearing <= 0.0) {
                airspeed_ref = wind_speed;
            }
            else {
                const double airsp_dot_bearing = 0.0; // right angle to the bearing line gives minimum airspeed usage, zero forward ground speed
                airspeed_ref = fabs(wind_cross_bearing);
            }
        }
        else {
            // infeasible
            airspeed_ref = (en_wind_excess_regulation_) ? airspeed_max_ : airspeed_nom_;;
        }
    }

    return airspeed_ref;
} // calcRefAirspeed

void NPFG::adjustRefAirVelForCurvature(Eigen::Vector2d &air_vel_ref, const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &unit_path_tangent,
    const double wind_speed, const double airspeed, const double feas, const double track_proximity, const double p_gain_adj,
    bool use_backwards_solution)
{
    double wind_cross_bearing, wind_dot_bearing;
    trigWindToBearing(wind_cross_bearing, wind_dot_bearing, wind_vel, unit_path_tangent);

    if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed, wind_speed)) {
        double airsp_dot_bearing = projectAirspOnBearing(airspeed, wind_cross_bearing);
        if (use_backwards_solution) {
            airsp_dot_bearing *= -1.0;
        }
        const double ground_speed = wind_dot_bearing + airsp_dot_bearing;
        const double feas0 = calcBearingFeas(wind_cross_bearing, wind_dot_bearing, airspeed, wind_speed);

        const double sin_eta_curv = track_proximity * feas * feas0 * ground_speed * path_curvature_ / airspeed / p_gain_adj * (1.0 + wind_dot_bearing / airsp_dot_bearing);
        const double cos_eta_curv = sqrt(std::max(1.0 - sin_eta_curv * sin_eta_curv, 0.0));

        air_vel_ref = rotate2d(sin_eta_curv, cos_eta_curv, air_vel_ref);
    }
} // adjustRefAirVelForCurvature

double NPFG::calcLateralAccel(const Eigen::Vector2d &air_vel, const Eigen::Vector2d &air_vel_ref, const double airspeed, const double p_gain_adj)
{
    const double dot_air_vel_err = air_vel(0) * air_vel_ref(0) + air_vel(1) * air_vel_ref(1);
    const double cross_air_vel_err = air_vel(0) * air_vel_ref(1) - air_vel(1) * air_vel_ref(0);
    if (dot_air_vel_err < 0.0) {
        return p_gain_adj * ((cross_air_vel_err < 0.0) ? -airspeed * airspeed : airspeed * airspeed);
    }
    else {
        return p_gain_adj * cross_air_vel_err;
    }
} // calcLateralAccel

bool NPFG::backwardsSolutionOK(const double wind_speed, const double airspeed, const double min_ground_speed, const double track_error)
{
    if (en_backwards_solution_ && airspeed < wind_speed) {
        // must satisfy:
        // 1) we are within the turn distance
        bool cond1 = wind_speed * M_PI / nom_heading_rate_ > track_error;
        // 2) the backwards solution satisfies the minimum ground speed at ALL bearings
        //    worst case: wind_dot_bearing = wind_speed; airsp_dot_bearing = -airspeed; compare to min_ground_speed_e_max_
        //    ^if allowing airspeed or min ground speed to vary, could cause oscillations in roll ref. if wind_speed is not stably in excess, similar problem
//        bool cond2 = wind_speed - airspeed >= std::max(min_ground_speed_g_, min_ground_speed_e_max_);
        // NOTE: since condition 2 is disabled, min ground sp may not always be maintained
        return cond1; // && cond2;
    }
    else {
        return false;
    }
} // backwardsSolutionOK

void NPFG::evaluate(const Eigen::Vector2d &ground_vel, const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &track_error_vec, const Eigen::Vector2d &unit_path_tangent)
{
    const double ground_speed = ground_vel.norm();

    Eigen::Vector2d air_vel = ground_vel - wind_vel;
    const double airspeed = air_vel.norm();

    const double wind_speed = wind_vel.norm();

    const double track_error = track_error_vec.norm();
    Eigen::Vector2d unit_track_error;
    if (track_error < EPSILON) {
        unit_track_error = track_error_vec;
    }
    else {
        unit_track_error = track_error_vec / track_error;
    }

    track_error_bound_ = calcTrackErrorBound(ground_speed);

    const double normalized_track_error = constrain(fabs(track_error / track_error_bound_), 0.0, 1.0);

    const double look_ahead_angle = calcLookAheadAngle(normalized_track_error);

    double track_proximity, inv_track_proximity;
    bearing_vec_ = calcBearingVec(track_proximity, inv_track_proximity, unit_track_error, unit_path_tangent, look_ahead_angle);

    // *feedback airspeed for gain adjustment
    p_gain_adj_ = adjustPGain(wind_speed / airspeed, inv_track_proximity);

    double wind_cross_bearing, wind_dot_bearing;
    trigWindToBearing(wind_cross_bearing, wind_dot_bearing, wind_vel, bearing_vec_);

    feas_ = calcBearingFeas(wind_cross_bearing, wind_dot_bearing, airspeed, wind_speed);

    // adjust along-bearing minimum ground speed for constant set point and/or track error
    double min_ground_speed = calcMinGroundSpeed(normalized_track_error);

    bool use_backwards_solution = backwardsSolutionOK(wind_speed, airspeed, min_ground_speed, track_error);

    Eigen::Vector2d air_vel_ref;
    if (feed_forward_air_vel_) {
        // check backwards solution for *nominal airspeed
        bool use_backwards_solution_nom = backwardsSolutionOK(wind_speed, airspeed_nom_, min_ground_speed, track_error);
        // get the heading reference assuming any airspeed incrementing is instantaneously achieved
        air_vel_ref = calcRefAirVelocityFF(wind_vel, bearing_vec_, wind_cross_bearing, wind_dot_bearing, wind_speed, min_ground_speed, use_backwards_solution_nom);
        airspeed_ref_ = air_vel_ref.norm();
        air_vel_ref = air_vel_ref * airspeed / airspeed_ref_; // re-scale with current airspeed for following feedback logic
    }
    else {
        air_vel_ref = calcRefAirVelocityFB(wind_vel, bearing_vec_, wind_cross_bearing, wind_dot_bearing, wind_speed, airspeed, use_backwards_solution);
        airspeed_ref_ = calcRefAirspeed(wind_vel, bearing_vec_, wind_cross_bearing, wind_dot_bearing, wind_speed, min_ground_speed);
    }

    // air velocity curvature adjustment -- feedback
    adjustRefAirVelForCurvature(air_vel_ref, wind_vel, unit_path_tangent, wind_speed, airspeed, feas_, track_proximity, p_gain_adj_, use_backwards_solution);

    lateral_accel_ = calcLateralAccel(air_vel, air_vel_ref, airspeed, p_gain_adj_);
} // evaluate

} // namespace fw_nmpc
