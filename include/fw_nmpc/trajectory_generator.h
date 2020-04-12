#ifndef TRAJECTOR_GENERATOR_H_
#define TRAJECTOR_GENERATOR_H_

#include <fw_nmpc/guidance/npfg.h>
#include <fw_nmpc/guidance/pwqg.h>
#include <Eigen/Eigen>

namespace fw_nmpc {

enum PathTypes { // TODO: eventually move to independent path manager
    LOITER = 0,
    LINE
}; // path types

// path params // TODO: eventually move to independent path manager
struct PathSetpoint {
    PathTypes type;
    Eigen::Vector3d pos;    // loiter: center of circle; line: pt. on line
    Eigen::Vector3d dir;    // loiter: unused; line: unit direction vector
    double signed_radius;   // loiter: signed radius (positive = clockwise); line: unused
};

enum MCLongitudinalModes {
    FPA = 0, // pitch stick controls flight path angle
    ALTITUDE, // pitch stick controls altitude
    TERRAIN_ALTITUDE // pitch stick controls altitude relative to terrain
}; // manual longitudinal control modes

enum MCLateralModes {
    ROLL = 0, // roll stick controls roll setpoint
    HEADING // roll stick controls deviation from current heading
}; // manual lateral-directional control modes

// manual control input
struct ManualControlInput {
    double x;
    double y;
    double z;
    double r;
};

// manual control setpoint
struct ManualControlSetpoint {
    bool enabled;
    MCLongitudinalModes lon_mode;
    MCLateralModes lat_mode;
    double airspeed;
    double heading;
    double fpa;
    double roll;
    double alt;
    double rel_alt;
};

/*
    TrajectoryGenerator class
    Generate pose trajectories for fixed-wing UAVs
*/
class TrajectoryGenerator {

    public:

        TrajectoryGenerator();

        enum TGMode {
            GUIDE_TO_PATH = 0, // follow guidance to path
            GUIDE_TO_PATH_FROM_CURRENT_HEADING // follow guidance to path starting from current heading
        }; // trajectory generation mode

        // NOTE: GUIDE_TO_PATH mode should NOT be used until we have a way to handle the rapid heading oscillations that can happen in excess wind scenarios

        // sets

        void enableOffTrackRollFF(bool en) { en_off_track_roll_ff_ = en; }
        void setMaxDeltaHeading(const double delta_heading) { max_delta_heading_ = delta_heading; }
        void setMaxDeltaAlt(const double delta_alt) { max_delta_alt_ = delta_alt; }
        void setDT(double dt) { dt_ = dt; }
        void setMode(TGMode mode) { tg_mode_ = mode; }
        void setCrossErrThres(double thres) { cross_err_thres_ = thres; }
        void setTrackErrThres(double thres) { track_err_thres_ = thres; }

        // guidance parameters

        void setNPFGParams(bool en_min_ground_speed, const double min_ground_speed_g, const double min_ground_speed_e_max,
            const double nte_fraction, const double p_gain, const double time_const, const double wind_ratio_buf);

        void setPWQGParams(const double fix_vert_pos_err_bnd, const double time_const, const double fpa_app_max, const double vert_pos_err_bnd);

        // manual control

        void updateManualSetpoint(ManualControlSetpoint &setpoint, const ManualControlInput &input,
            const double veh_pos_d, const double heading, const double terr_alt, const double airsp_min, const double airsp_max,
            const double roll_lim);

        double getManualFPASetpoint(double &err_lon, double *jac_fpa_sp, const ManualControlSetpoint &manual_control_sp,
            const double terr_alt, const double veh_pos_d, const double ground_speed, const double airspeed, const double wind_vel_d);

        // trajectory generation

        void genTrajectoryToPath2D(Eigen::Ref<Eigen::MatrixXd> pos_traj, Eigen::Ref<Eigen::MatrixXd> airsp_traj,
            Eigen::Ref<Eigen::MatrixXd> heading_traj, Eigen::Ref<Eigen::MatrixXd> roll_traj, const int len_traj,
            Eigen::Vector2d pos_0, const Eigen::Vector2d &ground_vel_0, const double airspeed_0, const double heading_0, const Eigen::Vector2d &wind_vel,
            const double airspeed_nom, const double airspeed_max, const PathSetpoint &path_sp, const double roll_lim_rad);

        double evaluateLongitudinalGuidance(double &err_lon, double *jac_fpa_sp, const PathSetpoint &path_sp, const Eigen::Vector3d &veh_pos,
            const double ground_speed, const double airspeed, const double wind_vel_d);

        // path following calculations

        static double calcRefPoseOnLine(Eigen::Vector3d &closest_pt_on_path, const Eigen::Vector3d &pt_on_line, const Eigen::Vector3d &unit_dir_vec, const Eigen::Vector3d& veh_pos);

        static double calcRefPoseOnLoiter(Eigen::Vector3d &closest_pt_on_path, Eigen::Vector3d &unit_path_tangent, const Eigen::Vector3d &circle_center, const double signed_radius,
            const Eigen::Vector3d& veh_pos, const Eigen::Vector3d& veh_vel, const double v_lat);

    private:

        static constexpr double MANUAL_CONTROL_DZ = 0.06; // dead-zone for stick on manual control
        static constexpr double INV_ONE_MINUS_DZ = 1.0 / (1.0 - MANUAL_CONTROL_DZ);
        static constexpr double INV_ONE_G = 1.0 / 9.81;

        NPFG npfg_;
        PWQG pwqg_;

        bool mc_was_disabled_;
        bool mc_lon_mode_changed_;
        bool mc_lat_mode_changed_;
        MCLongitudinalModes mc_last_lon_mode_;
        MCLateralModes mc_last_lat_mode_;

        double heading_hold_;
        double alt_hold_;
        double max_delta_heading_;
        double max_delta_alt_;

        double dt_;

        double cross_err_thres_;
        double track_err_thres_;

        TGMode tg_mode_;
        bool en_off_track_roll_ff_;

        // path following calculations

        double calcRefPoseOnLoiter2D(Eigen::Vector2d &closest_pt_on_path, Eigen::Vector2d &unit_path_tangent,
            const Eigen::Vector2d &circle_center, const double signed_radius, const Eigen::Vector2d& veh_pos, const Eigen::Vector2d& veh_vel, const double veh_speed);
        double calcRefPoseOnLine2D(Eigen::Vector2d &closest_pt_on_path, const Eigen::Vector2d &pt_on_line, const Eigen::Vector2d &unit_dir_vec, const Eigen::Vector2d& veh_pos);

}; // class TrajectoryGenerator

} // namespace fw_nmpc

#endif // TRAJECTOR_GENERATOR_H_
