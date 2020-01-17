/*******************************************************************************
 * Copyright (c) 2018, Thomas Stastny, Autonomous Systems Lab (ASL), ETH Zurich,
 * Switzerland
 *
 * You can contact the author at <thomas.stastny@mavt.ethz.ch>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3) Neither the name of ETHZ-ASL nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef _FW_NMPC_H
#define _FW_NMPC_H

// ROS
#include <ros/ros.h>
#include <ros/console.h>

// messages
#include <fw_ctrl/NMPCAuxOut.h>
#include <fw_ctrl/NMPCControls.h>
#include <fw_ctrl/NMPCInfo.h>
#include <fw_ctrl/NMPCMeasurements.h>
#include <fw_ctrl/NMPCObjNRef.h>
#include <fw_ctrl/NMPCObjRef.h>
#include <fw_ctrl/NMPCOnlineData.h>
#include <fw_ctrl/NMPCStates.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Grid map
#include <grid_map_ros/grid_map_ros.hpp>

// tf
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

// Eigen
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

// ACADO
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <fw_ctrl/fw_ctrlConfig.h>
#include <fw_ctrl/controlConfig.h>
#include <fw_ctrl/guidanceConfig.h>
#include <fw_ctrl/soft_constraintsConfig.h>

#define NX ACADO_NX     // Number of differential state variables
#define NU ACADO_NU     // Number of control inputs
#define NOD ACADO_NOD   // Number of online data values
#define NY ACADO_NY     // Number of measurements/references on nodes 0..N - 1
#define NYN ACADO_NYN   // Number of measurements/references on node N
#define N ACADO_N       // Number of intervals in the horizon
#define NOCC 6          // Number of stored occlusion attributes

#define MAX_SIZE_TERR_ARRAY 300000      // maximum allowed size of the local terrain map
#define LEN_SLIDING_WINDOW_MAX ACADO_N  // maximum length of sliding window
#define MIN_EXPONENTIAL_COST 1.0e-5     // minimum exponential cost at input arg = 1 (for soft constraints)

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

#define DEG_TO_RAD 0.017453292519943    // degrees to radians XXX: is there really no other function already built in for this?
#define EPSILON 0.000001                // used for triangle checks

#define AIR_GAS_CONST 287.1             // [J/(kg*K)]
#define ABSOLUTE_NULL_CELSIUS -273.15   // [C]
#define ONE_G 9.81                      // [m/s^2]

namespace fw_nmpc {

/* indexing enumeration */

enum IndexStates {
    IDX_X_POS = 0,
    IDX_X_V = 3,
    IDX_X_GAMMA,
    IDX_X_XI,
    IDX_X_PHI,
    IDX_X_THETA,
    IDX_X_NPROP
}; // states

enum IndexControls {
    IDX_U_U_T = 0,
    IDX_U_PHI_REF,
    IDX_U_THETA_REF
}; // controls

enum IndexOutputs {
    IDX_Y_VN = 0,
    IDX_Y_VE,
    IDX_Y_VD,
    IDX_Y_V,
    IDX_Y_PHI,
    IDX_Y_THETA,
    IDX_Y_SOFT_AOA,
    IDX_Y_SOFT_H,
    IDX_Y_SOFT_R,
    IDX_Y_U_T,
    IDX_Y_PHI_REF,
    IDX_Y_THETA_REF
}; // outputs

enum IndexOnlineData {
    IDX_OD_RHO = 0, // online parameters
    IDX_OD_W,
    IDX_OD_TAU_PHI = 4,
    IDX_OD_TAU_THETA,
    IDX_OD_K_PHI,
    IDX_OD_K_THETA,
    IDX_OD_TAU_N,
    IDX_OD_DELTA_F,
    IDX_OD_SOFT_AOA, // externally evaluated objectives and jacobians
    IDX_OD_JAC_SOFT_AOA,
    IDX_OD_SOFT_H = 13,
    IDX_OD_JAC_SOFT_H,
    IDX_OD_SOFT_R = 18,
    IDX_OD_JAC_SOFT_R
}; // online data
const int IDX_OD_OBJ = IDX_OD_SOFT_AOA; // starting index of externally evaluated objectives and jacobians

enum IndexOcclusionAttributes {
    IDX_OCC_POS = 0, // occlusion detection position
    IDX_OCC_NORMAL = 3 // occlusion normal
}; // occlusion attributes

enum IndexRadialOcclusionPriorities {
    IDX_PRIO_R_MAX = 0,
    IDX_PRIO_R_FWD,
    IDX_PRIO_R_LEFT,
    IDX_PRIO_R_RIGHT
}; // radial occlusion priorities

enum OffboardControlStatus {
    NOMINAL = 0,                // nmpc is running as expected
    INITIALIZATION_ERROR = 1,   // ACADO solver initialization failure
    PREPARATION_STEP_ERROR = 2, // failure in ACADO preparation step (model simulation function)
    FEEDBACK_STEP_ERROR = 3,    // failure in ACADO feedback step (qp solver -- qpOASES)
    BAD_SOLUTION = 4            // most often NaNs in solution - but could add high KKT value condition as well
}; // offboard control statuses

enum GuidancePathTypes {
    LOITER = 0,
    LINE
}; // guidance path types

/*
 * @brief fw_nmpc class
 *
 * class that implements fixed-wing nmpc
 */
class FwNMPC {

public:

    FwNMPC();

    /* callbacks */
    void actCb(const mavros_msgs::ActuatorControl::ConstPtr& msg);
    void gridMapCb(const grid_map_msgs::GridMap& msg);
    void sysStatusExtCb(const mavros_msgs::ExtendedState::ConstPtr& msg);
    void homePosCb(const mavros_msgs::HomePosition::ConstPtr& msg);
    void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
    void localPosCb(const nav_msgs::Odometry::ConstPtr& msg);
    void staticPresCb(const sensor_msgs::FluidPressure::ConstPtr& msg);
    void sysStatusCb(const mavros_msgs::State::ConstPtr& msg);
    void tempCCb(const sensor_msgs::Temperature::ConstPtr& msg);
    void windEstCb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);

    /* functions */
    void checkSubs();
    int nmpcIteration();
    int initNMPC();

    /* gets */
    double getLoopRate() const { return node_params_.nmpc_iteration_rate; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

    /* node handles */
    ros::NodeHandle nmpc_;                          // nmpc node handle
    ros::NodeHandle control_config_nh_;             // node handle for dynamic reconfig of control params
    ros::NodeHandle guidance_config_nh_;            // node handle for dynamic reconfig of guidance params
    ros::NodeHandle soft_constraints_config_nh_;    // node handle for dynamic reconfig of soft constraints params

    /* subscribers */
    ros::Subscriber act_sub_;
    ros::Subscriber grid_map_sub_;
    ros::Subscriber home_pos_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber local_pos_sub_;
    ros::Subscriber local_vel_sub_;
    ros::Subscriber static_pres_sub_;
    ros::Subscriber sys_status_sub_;
    ros::Subscriber sys_status_ext_sub_;
    ros::Subscriber temp_c_sub_;
    ros::Subscriber wind_est_sub_;

    /* publishers */
    ros::Publisher att_sp_pub_;
    ros::Publisher nmpc_aux_out_pub_;
    ros::Publisher nmpc_controls_pub_;
    ros::Publisher nmpc_guidance_path_pub_;
    ros::Publisher nmpc_info_pub_;
    ros::Publisher nmpc_meas_pub_;
    ros::Publisher nmpc_obj_ref_pub_;
    ros::Publisher nmpc_objN_ref_pub_;
    ros::Publisher nmpc_occ_detect_pub_;
    ros::Publisher nmpc_online_data_pub_;
    ros::Publisher nmpc_states_pub_;
    ros::Publisher nmpc_traj_pred_pub_;
    ros::Publisher obctrl_status_pub_;
    ros::Publisher thrust_pub_;

    ros::Publisher nmpc_obj_soft_aoa_pub_;
    ros::Publisher nmpc_obj_soft_h_pub_;
    ros::Publisher nmpc_obj_soft_r_pub_;

    /* server parameteres */

    // node parameters
    struct node_params {
        double nmpc_iteration_rate;
        double iteration_timestep;
        double nmpc_discretization;
        bool viz_en;
        bool ghost_en;
    };
    node_params node_params_;

    // vehicle parameters
    struct vehicle_params {
        double airsp_thres;
        double flaps_lim_rad;
    };
    vehicle_params vehicle_params_;

    // propeller parameters
    struct prop_params {
        double n_0;
        double n_max;
        double n_delta;
        double vp_min;
        double vp_max;
        double n_T0_vmin;
        double n_T0_vmax;
        double epsilon_T_rad;
    };
    prop_params prop_params_;

    // control-augmented model dynamics
    struct model_params {
        double tau_roll;
        double tau_pitch;
        double k_roll;
        double k_pitch;
        double tau_n_prop;
    };
    model_params model_params_;

    // control parameters
    struct control_params {
        double roll_lim_rad;
        double airsp_ref;
        bool enable_terrain_feedback;
        int len_slw;
        double tau_u;
        double tau_terr;
    };
    control_params control_params_;

    // soft constraint parameters
    struct soft_params {
        double sig_aoa_1;
        double sig_h_1;
        double k_r_offset;
        double k_delta_r;
        double sig_r_1;
    };
    soft_params soft_params_;

    /* dynamic reconfigure */

    // The dynamic reconfigure server + callback for control parameters
    dynamic_reconfigure::Server<fw_ctrl::controlConfig> serverControl;
    dynamic_reconfigure::Server<fw_ctrl::controlConfig>::CallbackType f_control;

    // The dynamic reconfigure server + callback for guidance parameters
    dynamic_reconfigure::Server<fw_ctrl::guidanceConfig> serverGuidance;
    dynamic_reconfigure::Server<fw_ctrl::guidanceConfig>::CallbackType f_guidance;

    // The dynamic reconfigure server + callback for soft constraint parameters
    dynamic_reconfigure::Server<fw_ctrl::soft_constraintsConfig> serverSoftConstraints;
    dynamic_reconfigure::Server<fw_ctrl::soft_constraintsConfig>::CallbackType f_soft_constraints;

    void parametersCallbackControl(const fw_ctrl::controlConfig &config, const uint32_t& level);
    void parametersCallbackGuidance(const fw_ctrl::guidanceConfig &config, const uint32_t& level);
    void parametersCallbackSoftConstraints(const fw_ctrl::soft_constraintsConfig &config, const uint32_t& level);

    /* functions */

    // math functions //XXX: these are duplicates from lsq_objective.c .. find a way to share.
    int constrain_int(int x, int xmin, int xmax);
    double constrain_double(double x, double xmin, double xmax);
    void cross(double *v, const double v1[3], const double v2[3]);
    double dot(const double v1[3], const double v2[3]);

    // helper functions
    double calcAirDensity();
    void calculate_speed_states(double *speed_states, const double v, const double gamma, const double xi, const double w_n, const double w_e, const double w_d); //XXX: another duplicate
    double flapsToRad(const double flaps_normalized);
    void ll2NE(double &north, double &east, const double lat, const double lon, const double lat_origin, const double lon_origin);
    double mapPropSpeedToNormalizedThrot(const double n_prop, const double airsp, const double aoa);
    double mapPX4ThrotToNormalizedThrot(const double px4_throt, const double airsp, const double aoa);
    double mapNormalizedThrotToPropSpeed(const double throt, const double airsp, const double aoa);
    double mapNormalizedThrotToPX4Throt(const double throt, const double airsp, const double aoa);
    void propagateVirtPropSpeed(const double throt, const double airsp, const double aoa);
    double unwrapHeading(const double yaw_meas);

    // publishers
    void publishControls(const double u_T, const double phi_ref, const double theta_ref);
    void publishNMPCInfo(ros::Time t_iter_start, uint64_t t_ctrl, uint64_t t_preeval, uint64_t t_prep, uint64_t t_fb);
    void publishNMPCStates();
    void publishNMPCVisualizations();
    void populateOcclusionMarkerArray(visualization_msgs::MarkerArray::Ptr occ_detections_msg, int &marker_counter, const int detection_count, const int size_list, int *occ_detected, double *occ_attributes);

    // objective functions
    void preEvaluateObjectives();
    void updateObjectiveParameters();
    void shiftOcclusionSlidingWindow();
    void castRays(const double *terrain_data);
    void sumOcclusionDetections();
    void evaluateExternalObjectives(const double *terrain_data);
    void prioritizeObjectives();
    void lookup_terrain_idx(const double pos_n, const double pos_e, const double pos_n_origin, const double pos_e_origin, const int map_height, const int map_width, const double map_resolution, int *idx_q, double *dn, double *de);
    int intersect_triangle(double *d_occ, double *p_occ, double *n_occ, const double r0[3], const double v_ray[3], const double p1[3], const double p2[3], const double p3[3], const int v_dir);
    int castray(double *r_occ, double *p_occ, double *n_occ, double *p1, double *p2, double *p3, const double r0[3], const double r1[3], const double v[3], const double pos_n_origin, const double pos_e_origin, const int map_height, const int map_width, const double map_resolution, const double *terr_map);
    void calculate_velocity_reference(double *v_ref, double *e_lat, double *e_lon, const double *states, const double *path_reference, const double *guidance_params, const double *speed_states, const double *jac_sig_r, const double prio_r, const int path_type);
    void jacobian_sig_h_lin(double *jac, const double de, const double delta_h, const double delta_y, const double  h1, const double h12, const double h2, const double h3, const double h34, const double h4, const double log_sqrt_w_over_sig1_h, const double sgn_e, const double sgn_n, const double map_resolution, const double xi);
    void jacobian_sig_h_exp(double *jac, const double de, const double delta_h, const double delta_y, const double h1, const double h12, const double h2, const double h3, const double h34, const double h4, const double log_sqrt_w_over_sig1_h, const double sgn_e, const double sgn_n, const double sig_h, const double map_resolution, const double xi);
    void jacobian_r_unit(double *jac, const double delta_r, const double gamma, const double k_delta_r, const double k_r_offset, const double n_occ_e, const double n_occ_h, const double n_occ_n, const double r_unit, const double v, const double v_ray_e, const double v_ray_h, const double v_ray_n, const double v_rel, const double xi);
    void calculate_aoa_objective(double *sig_aoa, double *jac_sig_aoa, double *prio_aoa, const double *states, const double *aoa_params);
    void calculate_height_objective(double *sig_h, double *jac_sig_h, double *prio_h, double *h_terr, const double *states, const double *terr_params, const double terr_local_origin_n, const double terr_local_origin_e, const int map_height, const int map_width, const double map_resolution, const double *terr_map);
    void calculate_radial_objective(double *sig_r, double *jac_sig_r, double *r_occ, double *p_occ, double *n_occ, double *prio_r, int *occ_detected, const double *v_ray, const double *states, const double *speed_states, const double *terr_params, const double terr_local_origin_n, const double terr_local_origin_e, const int map_height, const int map_width, const double map_resolution, const double *terr_map);
    void add_unit_radial_distance_and_gradient(double *jac_r_unit, double *r_unit_min, bool *f_min, int *occ_count, double *p_occ, double *n_occ, const double *states, const double *speed_states, const double *terr_params);
    void get_occ_along_gsp_vec(double *p_occ, double *n_occ, double *r_occ, int *occ_detected, const double *states, const double *speed_states, const double *terr_params, const double terr_local_origin_n, const double terr_local_origin_e, const int map_height, const int map_width, const double map_resolution, const double *terr_map);
    void filterControlReference();
    void filterTerrainCostJacobian();

    // NMPC functions
    void applyControl();            // apply current optimal control
    void updateAcadoConstraints();  // update ACADO constraints
    void updateAcadoOD();           // update ACADO online data
    void updateAcadoW();            // update ACADO weighting matrices
    void updateAcadoX0();           // update ACADO state measurements
    void updateAcadoY();            // update ACADO references

    // initialization
    void initACADOVars();
    void initHorizon();
    int initParameters();

    // ros node functions
    void shutdown();

    /* conversions */
    tf::Quaternion ned_enu_q_;                  // ned to enu (or visa versa) quaternion
    tf::Quaternion aircraft_baselink_q_;        // aircraft to baselink (or visa versa) quaternion
    Eigen::Vector3d translate_world_to_home_;   // XXX: for now.. ground truth to "map" frame translation

    /* pixhawk states / estimates */
    double flaps_normalized_;   // normalized flaps setting
    double home_lat_;           // home position latitude [deg]
    double home_lon_;           // home position longitude [deg]
    double home_alt_;           // home position altitude (absolute) [m]
    uint8_t landed_state_;      // landed state
    bool offboard_mode_;        // system is in offboard control mode
    double px4_throt_;          // throttle from pixhawk
    std::string px4_mode_;      // current mode received from PX4
    double static_pres_;        // static pressure [Pa]
    double temp_c_;             // temperature [C]
    Eigen::Vector3d x0_pos_;    // local position (ned) [m]
    Eigen::Vector3d x0_vel_;    // local velocity (ned) [m]
    Eigen::Vector3d x0_euler_;  // attitude (euler angles - RPY) [rad]
    Eigen::Vector3d x0_wind_;   // wind estimate (ned) [m/s]

    /* virtual states */
    double n_prop_virt_;        // virtual propeller speed state [rps]
    float last_unwrapped_yaw_;  // the last unwrapped yaw estimate [rad]
    int wrap_counter_;          // counts the number of times we've wrapped the yaw angle (and in which direction -- signed)

    /* grid map */
    grid_map::GridMap terrain_map_;
    double terr_array_[MAX_SIZE_TERR_ARRAY];
    double terr_local_origin_n_;
    double terr_local_origin_e_;
    int map_height_;
    int map_width_;
    double map_resolution_;

    /* solver matrices */
    Eigen::Matrix<double, ACADO_NX, 1> x0_;             // measured states
    Eigen::Matrix<double, ACADO_NU, 1> u_;              // currently applied controls
    Eigen::Matrix<double, ACADO_NU, ACADO_N> u_ref_;    // control reference horizon
    Eigen::Matrix<double, ACADO_NY, ACADO_N> y_;        // objective references
    Eigen::Matrix<double, ACADO_NYN, 1> yN_;            // end term objective references
    Eigen::Matrix<double, ACADO_NY, 1> inv_y_scale_sq_; // inverse output scale squared diagonal
    Eigen::Matrix<double, ACADO_NY, 1> w_;              // weight diagonal (before scaling / prioritization)
    Eigen::Matrix<double, ACADO_NOD, ACADO_N+1> od_;    // online data
    Eigen::Matrix<double, ACADO_NU, 1> lb_;             // lower bound of control constraints
    Eigen::Matrix<double, ACADO_NU, 1> ub_;             // upper bound of control constraints

    /* sliding window of occlusions */
    int len_sliding_window_;                                        // sliding window length
    int occ_detected_fwd_slw_[ACADO_N+LEN_SLIDING_WINDOW_MAX];      // sliding window of forward occlusion detections
    int occ_detected_left_[ACADO_N];                                // list of left side occlusion detections
    int occ_detected_right_[ACADO_N];                               // list of right side occlusion detections
    double occ_left_[ACADO_N * NOCC];                               // left side occlusion attributes (row-major format)
    double occ_right_[ACADO_N * NOCC];                              // right side occlusion attributes (row-major format)
    double occ_fwd_slw_[(ACADO_N+LEN_SLIDING_WINDOW_MAX) * NOCC];   // sliding window of forward occlusion attributes (row-major format)
    int occ_count_total_fwd_;                                       // total number of occlusion detections
    int occ_count_total_left_;                                      // total number of occlusion detections
    int occ_count_total_right_;                                     // total number of occlusion detections

    /* parameters */
    double guidance_params_[5];         // guidance parameters
    double aoa_params_[5];              // soft angle of attack parameters
    double terrain_params_[10];         // soft terrain parameters
    double log_sqrt_w_over_sig1_r_;     // XXX: these two are the only ones currently needed outside the lsq_objective.c file..
    double one_over_sqrt_w_r_;          // XXX: these two are the only ones currently needed outside the lsq_objective.c file..

    /* path */
    double path_reference_[5];  // path reference parameters
    int path_type_;             // 0 = constant altitude loiter, 1 = line

    /* objective priorities */
    Eigen::Matrix<double, ACADO_N+1, 1> prio_aoa_;  // soft angle of attack priority
    Eigen::Matrix<double, ACADO_N+1, 1> prio_h_;    // soft nadir terrain priority
    Eigen::Matrix<double, ACADO_N+1, 4> prio_r_;    // soft radial terrain priority

    /* timing */
    ros::Time t_last_ctrl_; // time since last control action [s]

    /* state machine */
    bool home_pos_valid_;                   // received valid home position
    bool px4_connected_;                    // PX4 FCU is connected
    OffboardControlStatus obctrl_status_;   // offboard control status
    int err_code_preparation_step_;         // detailed error code for ACADO preparation step
    int err_code_feedback_step_;            // detailed error code for ACADO feedback step
    bool re_init_horizon_;                  // re-initialize the horizon
    bool grid_map_valid_;
};

} // namespace fw_nmpc

#endif
