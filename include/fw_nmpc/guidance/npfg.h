#ifndef NPFG_H_
#define NPFG_H_

#include <Eigen/Eigen>

namespace fw_nmpc {

/*
    NPFG class
    Lateral-directional nonlinear path following guidance logic with excess wind handling
 */
class NPFG {

    public:

        NPFG();

        // gets
        double getAirspeedRef() { return airspeed_ref_; }
        Eigen::Vector2d getAirVelRef() { return air_vel_ref_; }
        double getBearingFeas() { return feas_; }
        Eigen::Vector2d getBearingVec() { return bearing_vec_; }
        double getLateralAccel() { return lateral_accel_; }
        double getPGainAdj() { return p_gain_adj_; }
        double getTrackErrorBound() { return track_error_bound_; }
        double getGroundSpMinE() { return min_ground_speed_e_; }

        // sets
        void enableBackwardsSolution(double en) { en_backwards_solution_ = en; }
        void enableFeedForwardAirVelRef(double en) { feed_forward_air_vel_ = en; }
        void enableMinGroundSpeed(double en) { en_min_ground_speed_ = en; }
        void enableTrackKeeping(double en) { en_track_keeping_ = en; }
        void enableWindExcessRegulation(double en) { en_wind_excess_regulation_ = en; }
        void setAirspeedMax(double airspeed_max) { airspeed_max_ = airspeed_max; }
        void setAirspeedNom(double airspeed_nom) { airspeed_nom_ = airspeed_nom; }
        void setMinGroundSpeed(double min_ground_speed) { min_ground_speed_g_ = std::max(min_ground_speed, 0.0); }
        void setMinGroundSpeedEMax(double min_ground_speed_e_max) { min_ground_speed_e_max_ = std::max(min_ground_speed_e_max, 0.0); }
        void setNominalHeadingRate(double nom_heading_rate) { nom_heading_rate_ = std::max(nom_heading_rate, 0.01); }
        void setNTEFraction(double nte_fraction) { inv_nte_fraction_ = 1.0 / std::max(nte_fraction, 0.01); }
        void setPathCurvature(double curvature) { path_curvature_ = curvature; }
        void setPGain(double p_gain) { p_gain_ = p_gain; }
        void setTimeConstant(double time_const) { time_const_ = time_const; }
        void setWindRatioBuf(double wind_ratio_buf) { wind_ratio_buf_ = wind_ratio_buf; }

        // public functions
        void evaluate(const Eigen::Vector2d &ground_vel, const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &track_error_vec, const Eigen::Vector2d &unit_path_tangent);

    private:

        static constexpr double LAMBDA_CO = 0.02;                           // linear finite cut-off angle [rad] (= approx. 1 deg)
        static constexpr double ONE_OVER_S_LAMBDA_CO = 50.003333488895450;  // 1/sin(lambda_co)
        static constexpr double M_CO = 2499.833309998360;                   // linear finite cut-off slope = cos(lambda_co)/sin(lambda_co)^2
        static constexpr double P_GAIN_MIN_MULT = 1.1;                      // multiplied safety factor to keep minimum gain above what is necessary for convergence
        static constexpr double EPSILON = 1.0e-4;

        double feas_;                   // bearing feasibility
        double lateral_accel_;          // lateral acceleration setpoint [m/s^2]
        double path_curvature_;         // path curvature

        double track_error_bound_;      // track error boundary [m]
        Eigen::Vector2d bearing_vec_;   // unit bearing vector

        double p_gain_;                 // multiplied proportional gain
        double p_gain_adj_;             // (possibly adjusted) proportional gain (accounting for minimum)
        double time_const_;             // look-ahead time constant [s]

        double airspeed_nom_;           // nominal airspeed reference [m/s]
        double airspeed_max_;           // maximum airspeed reference [m/s]
        double airspeed_ref_;           // airspeed reference [m/s]
        Eigen::Vector2d air_vel_ref_;   // air velocity reference [m/s]
        double wind_ratio_buf_;         // percentage buffer of bearing feasibility at lambda = 90 deg (NOTE: this is constant to avoid oscillations)

        double inv_nte_fraction_;       // 1 / ( fraction of the normalized track error for which we consider full ground speed compensation )
        double min_ground_speed_e_;     // track-error dependent minimum along-bearing ground speed [m/s]
        double min_ground_speed_e_max_; // minimum along-bearing ground speed at maximum track offset [m/s]
        double min_ground_speed_g_;     // minimum along-bearing ground speed [m/s]

        // airspeed reference compensation modes
        bool en_wind_excess_regulation_;    // enable wind excess regulation
        bool en_track_keeping_;             // enable track offset dependent minimum ground speed incrementing
        bool en_min_ground_speed_;          // enable user-set minimum ground speed maintenance
        bool feed_forward_air_vel_;         // use the compensated air velocity reference as a heading reference
        bool en_backwards_solution_;        // enable use of the backwards excess wind heading solution !!DANGER ZONE!!

        double nom_heading_rate_;           // maximum heading rate at nominal airspeed (user is responsible for computing externally) [rad/s]

        // private functions
        double calcTrackErrorBound(const double ground_speed);
        double calcLookAheadAngle(const double normalized_track_error);
        Eigen::Vector2d calcBearingVec(double &track_proximity, double &inv_track_proximity,
            const Eigen::Vector2d &unit_track_error, const Eigen::Vector2d &unit_path_tangent, const double look_ahead_angle);
        double adjustPGain(const double wind_ratio, const double normalized_track_error);
        double calcMinGroundSpeed(const double normalized_track_error);
        void trigWindToBearing(double &wind_cross_bearing, double &wind_dot_bearing, const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec);
        double projectAirspOnBearing(const double airspeed, const double wind_cross_bearing);
        int bearingIsFeasible(const double wind_cross_bearing, const double wind_dot_bearing, const double airspeed, const double wind_speed);
        double calcBearingFeas(const double wind_cross_bearing, const double wind_dot_bearing, const double airspeed, const double wind_speed);
        Eigen::Vector2d calcRefAirVelocityFF(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec,
            const double wind_cross_bearing, const double wind_dot_bearing, const double wind_speed, const double min_ground_speed, bool use_backwards_solution);
        Eigen::Vector2d calcRefAirVelocityFB(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec,
            const double wind_cross_bearing, const double wind_dot_bearing, const double wind_speed, const double airspeed, bool use_backwards_solution);
        Eigen::Vector2d calcInfeasibleAirVelRef(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec, const double wind_speed, const double airspeed);
        double calcRefAirspeed(const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &bearing_vec,
            const double wind_cross_bearing, const double wind_dot_bearing, const double wind_speed, const double min_ground_speed);
        void adjustRefAirVelForCurvature(Eigen::Vector2d &air_vel_ref, const Eigen::Vector2d &wind_vel, const Eigen::Vector2d &unit_path_tangent,
            const double wind_speed, const double airspeed, const double feas, const double track_proximity, const double p_gain_adj, bool use_backwards_solution);
        double calcLateralAccel(const Eigen::Vector2d &air_vel, const Eigen::Vector2d &air_vel_ref, const double airspeed, const double p_gain_adj);
        bool backwardsSolutionOK(const double wind_speed, const double airspeed, const double min_ground_speed, const double track_error);

        friend class TrajectoryGenerator;
}; // class NPFG

} // namespace fw_nmpc

#endif // NPFG_H_
