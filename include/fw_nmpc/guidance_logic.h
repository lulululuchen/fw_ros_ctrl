#ifndef GUIDANCE_LOGIC_H_
#define GUIDANCE_LOGIC_H_

#include <Eigen/Eigen>

namespace fw_nmpc {

enum ManualControlTypes { //XXX: does dyn reconfig already include these enum definitions? can this be thrown away?
    DIRECTION = 0, // set roll (or bearing) and flight path angle (ground relative)
    ALTITUDE, // set roll (or bearing) and altitude
    TERRAIN_ALTITUDE // set roll (or bearing) and altitude relative to terrain
}; // manual control types

enum ManualControlLatInputs { //XXX: does dyn reconfig already include these enum definitions? can this be thrown away?
    ROLL = 0, // command roll angle
    BEARING_RATE // command bearing rate
}; // lateral-directional manual control inputs

enum GuidanceLogicTypes {
    PW_QUAD = 0, // piece-wise quadratic
    ARCTAN // arctangent
}; // guidance logic types

// manual control setpoint
struct ManualControlSetpoint {
    bool enabled;
    ManualControlTypes type;
    ManualControlLatInputs lat_input;
    double airspeed;
    double roll;
    double bearing;
    double bearing_rate;
    double unit_delta_alt_rate;
    double max_bearing_rate;
    Eigen::Vector3d unit_vel;
    double alt;
    double rel_alt;
};

enum PathTypes { // TODO: eventually move to independent path manager
    LOITER = 0,
    LINE
}; // path types

// path params // TODO: eventually move to independent path manager
struct PathSetpoint {
    PathTypes type;
    Eigen::Vector3d pos;
    double bearing;
    double fpa;
    double signed_radius;
};

/*
    GuidanceLogic class
    Library of various high-level fixed-wing guidance logic and associated functionality
 */
class GuidanceLogic {

    public:

        GuidanceLogic();

        // gets
        double getUnitZAppMax() { return unit_z_app_max_; };
        bool getUseOCCAsGuidance() { return use_occ_as_guidance_; };

        // sets
        void setFixedVertPosErrBound(const bool en) { fix_vert_pos_err_bnd_ = en; };
        void setFPAAppMax(const double fpa);
        void setGuidanceType(const GuidanceLogicTypes sel) { guidance_sel_ = sel; };
        void setLatTimeConstant(const double tc) { T_lat_ = tc; };
        void setLonTimeConstant(const double tc) { T_lon_ = tc; };
        void setUseOCCAsGuidance(const bool en) { use_occ_as_guidance_ = en; };
        void setVertPosErrBound(const double bnd) { vert_pos_err_bnd_ = bnd; };

        // functions
        void augmentTerrainCostToGuidance(Eigen::Vector3d &unit_ground_vel_sp, const Eigen::Vector3d &terr_cost_pos_gradient, const double inv_terrain_prio);
        void calculateUnitVelocityReference(Eigen::Vector3d &unit_ground_vel_sp, double &err_lat, double &err_lon, double &unit_err_lat,
            const Eigen::Vector3d &veh_pos, const Eigen::Vector3d &ground_vel, const double ground_sp_lat, const double terrain_alt,
            const ManualControlSetpoint &manual_control_sp, const PathSetpoint &path_sp);

    private:

        // functions
        double calculatePositionErrorBoundary(const double v, const double tc);
        void calculateReferencePoseOnLine(Eigen::Vector3d &closest_pt_on_path, Eigen::Vector3d &unit_path_tangent, const Eigen::Vector3d &pt_on_line, const double bearing, const double fpa_ref, const Eigen::Vector3d& veh_pos);
        void calculateReferencePoseOnLoiter(Eigen::Vector3d &closest_pt_on_path, Eigen::Vector3d &unit_path_tangent, const Eigen::Vector3d &circle_center, const double signed_radius, const Eigen::Vector3d& veh_pos, const Eigen::Vector3d& veh_vel, const double v_lat);
        void followManual(Eigen::Vector3d &unit_ground_vel_sp, double &err_lat, double &err_lon, double &unit_err_lat,
            const ManualControlSetpoint &manual_control_sp, const double terr_alt, const Eigen::Vector3d &veh_pos,
            const Eigen::Vector3d &ground_vel, const double &ground_sp_lat);
        void followPath(Eigen::Vector3d &unit_ground_vel_sp, double &e_lat, double &e_lon, double &e_lat_unit,
            const PathSetpoint &path_sp, const Eigen::Vector3d &veh_pos, const Eigen::Vector3d &veh_vel, const double ground_sp_lat);
        double longitudinalArctanVFGuidance(double &err_lon, const double d_ref, const double unit_path_tangent_d, const double fpa_ref, const double veh_pos_d, const double veh_vel_d);
        double longitudinalPwQuadGuidance(double &err_lon, const double d_ref, const double unit_path_tangent_d, const double veh_pos_d, const double veh_vel_d);
        void arctanVFGuidance(Eigen::Vector3d &unit_vel_sp, double &err_lat, double &err_lon, double &unit_err_lat, const Eigen::Vector3d &pt_on_path, const Eigen::Vector3d &unit_path_tangent, const double bearing, const double fpa_ref, const Eigen::Vector3d &veh_pos, const Eigen::Vector3d &veh_vel, const double v_lat);
        void pwQuadGuidance(Eigen::Vector3d &unit_vel_sp, double &err_lat, double &err_lon, double &unit_err_lat, const Eigen::Vector3d& pt_on_path, const Eigen::Vector3d& unit_path_tangent, const Eigen::Vector3d& veh_pos, const Eigen::Vector3d& veh_vel, const double v_lat);

        // guidance parameters
        bool fix_vert_pos_err_bnd_;
        double gamma_app_max_;
        GuidanceLogicTypes guidance_sel_;
        double T_lat_;
        double T_lon_;
        double unit_z_app_max_;
        bool use_occ_as_guidance_;
        double vert_pos_err_bnd_;
}; // class GuidanceLogic

} // namespace fw_nmpc

#endif // GUIDANCE_LOGIC_H_
