/*******************************************************************************
 * Copyright (c) 2019, Thomas Stastny, Autonomous Systems Lab (ASL), ETH Zurich,
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

#ifndef NONLINEAR_MPC_H_
#define NONLINEAR_MPC_H_

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
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float32.h>
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

// std
#include <string>

// ACADO
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

// fw_nmpc
#include <fw_nmpc/common/helpers.h>
#include <fw_nmpc/common/huber_constraint.h>
#include <fw_nmpc/guidance/npfg.h>
#include <fw_nmpc/guidance/pwqg.h>
#include <fw_nmpc/nonlinear_mpc_objectives.h>
#include <fw_nmpc/occlusion_detector.h>
#include <fw_nmpc/trajectory_generator.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <fw_ctrl/fw_ctrlConfig.h>
#include <fw_ctrl/controlConfig.h>
#include <fw_ctrl/manual_controlConfig.h>
#include <fw_ctrl/occlusion_detectionConfig.h>
#include <fw_ctrl/soft_constraintsConfig.h>
#include <fw_ctrl/trajectory_generationConfig.h>

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace fw_nmpc {

const int MAX_SIZE_TERR_ARRAY = 300000; // maximum allowed size of the local terrain map

/* indexing enumeration */

/*
    NonlinearMPC class
    Implements nonlinear model predictive controller for a fixed-wing UAV
 */
class NonlinearMPC {

    public:

        NonlinearMPC();

        /* callbacks */
        void actCb(const mavros_msgs::ActuatorControl::ConstPtr& msg);
        void gridMapCb(const grid_map_msgs::GridMap& msg);
        void sysStatusExtCb(const mavros_msgs::ExtendedState::ConstPtr& msg);
        void homePosCb(const mavros_msgs::HomePosition::ConstPtr& msg);
        void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
        void localPosCb(const nav_msgs::Odometry::ConstPtr& msg);
        void manCtrlCb(const mavros_msgs::ManualControl::ConstPtr& msg);
        void staticPresCb(const sensor_msgs::FluidPressure::ConstPtr& msg);
        void sysStatusCb(const mavros_msgs::State::ConstPtr& msg);
        void tempCCb(const sensor_msgs::Temperature::ConstPtr& msg);
        void windEstCb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);

        /* functions */
        void checkSubs();
        int nmpcIteration();
        int initializeNMPC();

        /* gets */
        double getLoopRate() const { return node_parameters_.nmpc_iteration_rate; }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:

        /* node handles */
        ros::NodeHandle nmpc_;                              // nmpc node handle
        ros::NodeHandle control_config_nh_;                 // node handle for dynamic reconfig of control params
        ros::NodeHandle manual_control_config_nh_;          // node handle for dynamic reconfig of manual control params
        ros::NodeHandle occlusion_detection_config_nh_;     // node handle for dynamic reconfig of occlusion detection params
        ros::NodeHandle soft_constraints_config_nh_;        // node handle for dynamic reconfig of soft constraints params
        ros::NodeHandle trajectory_generation_config_nh_;   // node handle for dynamic reconfig of trajectory generation params

        /* subscribers */
        ros::Subscriber act_sub_;
        ros::Subscriber grid_map_sub_;
        ros::Subscriber home_pos_sub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber local_pos_sub_;
        ros::Subscriber man_ctrl_sub_;
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
        ros::Publisher nmpc_occ_normals_pub_;
        ros::Publisher nmpc_occ_planes_pub_;
        ros::Publisher nmpc_occ_rays_pub_;
        ros::Publisher nmpc_online_data_pub_;
        ros::Publisher nmpc_states_pub_;
        ros::Publisher nmpc_traj_pred_pub_;
        ros::Publisher nmpc_traj_ref_pub_;
        ros::Publisher obctrl_status_pub_;
        ros::Publisher thrust_pub_;
        ros::Publisher air_vel_ref_pub_;

        /* indexing */

        enum IndexStates { // TODO: change the greek symbols to the actual state names
            IDX_X_POS = 0,
            IDX_X_AIRSP = 3,
            IDX_X_FPA,
            IDX_X_HEADING,
            IDX_X_ROLL,
            IDX_X_PITCH,
            IDX_X_PROP_SP
        }; // states

        enum IndexControls {
            IDX_U_THROT = 0,
            IDX_U_ROLL_REF,
            IDX_U_PITCH_REF
        }; // controls

        enum IndexOutputs {
            IDX_Y_POS = 0,
            IDX_Y_AIRSP = 2,
            IDX_Y_FPA,
            IDX_Y_HEADING,
            IDX_Y_SOFT_AIRSP,
            IDX_Y_SOFT_AOA,
            IDX_Y_SOFT_HAGL,
            IDX_Y_SOFT_RTD,
            IDX_Y_THROT,
            IDX_Y_ROLL_REF,
            IDX_Y_PITCH_REF
        }; // outputs

        enum IndexOnlineData { // TODO: change the greek symbols to the actual state names
            IDX_OD_AIR_DENSITY = 0, // online parameters
            IDX_OD_WIND,
            IDX_OD_TAU_ROLL = 4,
            IDX_OD_TAU_PITCH,
            IDX_OD_K_ROLL,
            IDX_OD_K_PITCH,
            IDX_OD_TAU_PROP,
            IDX_OD_FLAPS,
            IDX_OD_FPA_REF,
            IDX_OD_JAC_FPA_REF,
            IDX_OD_HEADING_REF,
            IDX_OD_SOFT_AIRSP, // externally evaluated objectives and jacobians
            IDX_OD_JAC_SOFT_AIRSP,
            IDX_OD_SOFT_AOA,
            IDX_OD_JAC_SOFT_AOA,
            IDX_OD_SOFT_HAGL = 17,
            IDX_OD_JAC_SOFT_HAGL,
            IDX_OD_SOFT_RTD = 22,
            IDX_OD_JAC_SOFT_RTD
        }; // online data
        const int IDX_OD_OBJ = IDX_OD_SOFT_AIRSP; // starting index of externally evaluated objectives and jacobians
        const int LEN_JAC_SOFT_AIRSP = IDX_OD_SOFT_AOA - IDX_OD_JAC_SOFT_AIRSP; // how many states we are taking the jacobian w.r.t.
        const int LEN_JAC_SOFT_AOA = IDX_OD_SOFT_HAGL - IDX_OD_JAC_SOFT_AOA; // how many states we are taking the jacobian w.r.t.
        const int LEN_JAC_SOFT_HAGL = IDX_OD_SOFT_RTD - IDX_OD_JAC_SOFT_HAGL; // how many states we are taking the jacobian w.r.t.
        const int LEN_JAC_SOFT_RTD = 6; // how many states we are taking the jacobian w.r.t.

        enum OffboardControlStatus {
            NOMINAL = 0,                // nmpc is running as expected
            INITIALIZATION_ERROR = 1,   // ACADO solver initialization failure
            PREPARATION_STEP_ERROR = 2, // failure in ACADO preparation step (model simulation function)
            FEEDBACK_STEP_ERROR = 3,    // failure in ACADO feedback step (qp solver -- qpOASES)
            BAD_SOLUTION = 4            // most often NaNs in solution - but could add high KKT value condition as well
        }; // offboard control statuses

        enum OcclusionBuffers {
            IDX_OCC_FWD = 0,
            IDX_OCC_RIGHT,
            IDX_OCC_LEFT
        }; // occlusion buffers
        static const int NUM_OCC_BUF = IDX_OCC_LEFT+1; // number of occlusion buffers we are defining..

        /* parameter structs */

        // node parameters
        struct NodeParameters {
            double nmpc_iteration_rate;
            double iteration_timestep;
            double nmpc_discretization;
            bool viz_en;
            bool ghost_en;
        } node_parameters_;

        // vehicle parameters
        struct VehicleParameters {
            double airsp_thres;
            double airsp_min;
            double airsp_max;
            double flaps_lim_rad;
        } veh_parameters_;

        // propeller parameters
        struct PropellerParameters {
            double n_0;
            double n_max;
            double n_delta;
            double vp_min;
            double vp_max;
            double n_T0_vmin;
            double n_T0_vmax;
            double epsilon_T_rad;
        } prop_parameters_;

        // control-augmented model dynamics
        struct ModelParameters {
            double tau_roll;
            double tau_pitch;
            double k_roll;
            double k_pitch;
            double tau_prop;
        } model_parameters_;

        // control parameters
        struct ControlParameters {
            double roll_lim_rad;
            double airsp_ref;
            bool enable_terrain_feedback;
            double tau_terr;
            bool use_ff_roll_ref;
            bool use_floating_ctrl_ref;
            double tau_u;
            double fixed_pitch_ref;
            double fixed_throt_ref;
        } control_parameters_;

        /* dynamic reconfigure */

        // The dynamic reconfigure server + callback for control parameters
        dynamic_reconfigure::Server<fw_ctrl::controlConfig> serverControl;
        dynamic_reconfigure::Server<fw_ctrl::controlConfig>::CallbackType f_control;

        // The dynamic reconfigure server + callback for manual control configuration
        dynamic_reconfigure::Server<fw_ctrl::manual_controlConfig> serverManualControl;
        dynamic_reconfigure::Server<fw_ctrl::manual_controlConfig>::CallbackType f_manual_control;

        // The dynamic reconfigure server + callback for occlusion detection configuration
        dynamic_reconfigure::Server<fw_ctrl::occlusion_detectionConfig> serverOcclusionDetection;
        dynamic_reconfigure::Server<fw_ctrl::occlusion_detectionConfig>::CallbackType f_occlusion_detection;

        // The dynamic reconfigure server + callback for soft constraint parameters
        dynamic_reconfigure::Server<fw_ctrl::soft_constraintsConfig> serverSoftConstraints;
        dynamic_reconfigure::Server<fw_ctrl::soft_constraintsConfig>::CallbackType f_soft_constraints;

        // The dynamic reconfigure server + callback for trajectory generation parameters
        dynamic_reconfigure::Server<fw_ctrl::trajectory_generationConfig> serverTrajectoryGeneration;
        dynamic_reconfigure::Server<fw_ctrl::trajectory_generationConfig>::CallbackType f_trajectory_generation;

        void parametersCallbackControl(const fw_ctrl::controlConfig &config, const uint32_t& level);
        void parametersCallbackManualControl(const fw_ctrl::manual_controlConfig &config, const uint32_t& level);
        void parametersCallbackOcclusionDetection(const fw_ctrl::occlusion_detectionConfig &config, const uint32_t& level);
        void parametersCallbackSoftConstraints(const fw_ctrl::soft_constraintsConfig &config, const uint32_t& level);
        void parametersCallbackTrajectoryGeneration(const fw_ctrl::trajectory_generationConfig &config, const uint32_t& level);

        /* functions */

        // publishers
        void publishControls(const double u_T, const double phi_ref, const double theta_ref);
        void publishNMPCInfo(ros::Time t_iter_start, uint64_t t_ctrl, uint64_t t_preeval, uint64_t t_prep, uint64_t t_fb);
        void publishNMPCStates();
        void publishNMPCVisualizations();
        bool populateOcclusionLists(visualization_msgs::Marker::Ptr rays_msg, visualization_msgs::Marker::Ptr normals_msg, visualization_msgs::Marker::Ptr planes_msg);

        // aircarft state conversions / calculations
        void calculateSpeedStatesHorizon();
        double flapsToRad(const double flaps_normalized);
        double mapPropSpeedToNormalizedThrot(const double n_prop, const double airsp, const double aoa);
        double mapPX4ThrotToNormalizedThrot(const double px4_throt, const double airsp, const double aoa);
        double mapNormalizedThrotToPropSpeed(const double throt, const double airsp, const double aoa);
        double mapNormalizedThrotToPX4Throt(const double throt, const double airsp, const double aoa);
        void propagateVirtPropSpeed(const double throt, const double airsp, const double aoa);

        // objective pre-evaluation functions
        void detectOcclusions();
        void evaluateSoftAirspeedObjective();
        void evaluateSoftAOAObjective();
        void evaluateSoftHAGLObjective();
        void evaluateSoftRTDObjective();
        void findNearestRayOrigins();
        void preEvaluateObjectives();
        void setControlObjectiveReferences();
        void setManualControlReferences();
        void setOcclusionWindows();
        void setPathFollowingReferences();

        // NMPC functions
        void applyControl();                // apply current optimal control
        void filterControlReference();
        void filterTerrainCostJacobian();
        void prioritizeObjectives();
        void setObjectiveReferences();
        void updateAcadoConstraints();      // update ACADO constraints
        void updateAcadoOD();               // update ACADO online data
        void updateAcadoW();                // update ACADO weighting matrices
        void updateAcadoX0();               // update ACADO state measurements
        void updateAcadoY();                // update ACADO objective references
        void updateObjectiveParameters();

        // initialization
        void initializeVariables();
        void initializeHorizon();
        int initializeParameters();

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
        double static_pressure_pa_; // static pressure [Pa]
        double temperature_c_;      // temperature [C]
        Eigen::Vector3d x0_pos_;    // local position (ned) [m]
        Eigen::Vector3d x0_vel_;    // local velocity (ned) [m]
        Eigen::Vector3d x0_euler_;  // attitude (euler angles - RPY) [rad]
        Eigen::Vector3d x0_wind_;   // wind estimate (ned) [m/s]

        /* virtual states */
        double n_prop_virt_;        // virtual propeller speed state [rps]

        /* grid map */
        grid_map::GridMap terrain_map_;
        double map_array_[MAX_SIZE_TERR_ARRAY];
        double map_origin_north_;
        double map_origin_east_;
        int map_height_;
        int map_width_;
        double map_resolution_;

        /* solver matrices */
        Eigen::Matrix<double, ACADO_NX, 1> x0_;             // measured states
        Eigen::Matrix<double, ACADO_NU, 1> u_;              // currently applied controls
        Eigen::Matrix<double, ACADO_NY, ACADO_N+1> y_;      // objective references (including end term)
        Eigen::Matrix<double, ACADO_NY, 1> inv_y_scale_sq_; // inverse output scale squared diagonal
        Eigen::Matrix<double, ACADO_NY, 1> w_;              // weight diagonal (before scaling / prioritization)
        Eigen::Matrix<double, ACADO_NOD, ACADO_N+1> od_;    // online data
        Eigen::Matrix<double, ACADO_NU, 1> lb_;             // lower bound of control constraints
        Eigen::Matrix<double, ACADO_NU, 1> ub_;             // upper bound of control constraints

        /* occlusion detection */
        OcclusionDetector occ_[NUM_OCC_BUF];                    // multiple occlusion buffers
        int ray_casting_interval_;                              // interval between MPC nodes at which we cast rays
        int len_occ_data_;                                      // number of rays we cast per buffer rung
        int len_occ_window_;                                    // number of occlusions in sliding window used to generate local relative distance field
        int len_occ_buffer_;                                    // length of data history buffer
        int rtd_node_interval_;                                 // interval between MPC nodes at which we compute RTD costs/jacobians
        Eigen::Matrix<int, ACADO_N+1, 1> occ_window_start_;     // starting indices of each occlusion's window
        Eigen::Matrix<int, ACADO_N+1, 1> nearest_ray_origin_;   // closest ray origin to each MPC node
        double surfel_radius_;                                  // occlusion surfel radius [m] (=0 implies a point)

        /* reference trajectories */
        TrajectoryGenerator traj_gen_;                  // object containing guidance logic and relevant functionality for generating desired trajectories
        ManualControlSetpoint manual_control_sp_;       // structure for manual control setpoints
        ManualControlInput manual_control_input_;       // structure for raw manual control inputs
        PathSetpoint path_sp_;                          // structure for path setpoints

        /* external objective evaluation functions */
        NonlinearMPCObjectives ext_obj_;

        /* structs for efficiently pre-calculating multiply used quantities */

        // collection of velocity states
        struct SpeedStates {
            double air_vel[3];
            double ground_vel[3];
            double ground_sp_sq;
            double ground_sp;
            double inv_ground_sp;
            double unit_ground_vel[3];
            double ground_sp_lat_sq;
            double ground_sp_lat;
            double unit_ground_vel_lat[2];
        } spds_[ACADO_N+1];

        // terrain altitude at each node in the horizon
        Eigen::Matrix<double, ACADO_N+1, 1> terrain_alt_horizon_;

        /* soft constraints */
        ExponentialHuberConstraint huber_airsp_{"lower"};
        ExponentialHuberConstraint huber_aoa_p_{"upper"};
        ExponentialHuberConstraint huber_aoa_m_{"lower"};
        ExponentialHuberConstraint huber_hagl_{"lower"};
        struct HuberRTDParams {
            double constr_0;        // radial terrain distance offset [m]
            double delta_0;         // radial terrain distance delta (at zero relative velocity) [m]
            double constr_gain;     // radial terrain distance constraint gain
            double delta_gain;      // radial terrain distance delta gain
            double constr_scaler;   // radial terrain distance constraint scaler
            double delta_scaler;    // radial terrain distance delta scaler
        } huber_rtd_params_;

        /* objective priorities */
        Eigen::Matrix<double, ACADO_N+1, 1> inv_prio_airsp_;    // inverse priority of soft airspeed
        Eigen::Matrix<double, ACADO_N+1, 1> inv_prio_aoa_;      // inverse priority of soft angle of attack
        Eigen::Matrix<double, ACADO_N+1, 1> inv_prio_hagl_;     // inverse priority of soft height above ground level (HAGL)
        Eigen::Matrix<double, ACADO_N+1, 1> inv_prio_rtd_;      // inverse priority of soft radial terrain distance (RTD)

        /* timing */
        ros::Time t_last_ctrl_; // time of last control action [s]

        /* state machine */
        bool home_pos_valid_;                   // received valid home position
        bool px4_connected_;                    // PX4 FCU is connected
        OffboardControlStatus obctrl_status_;   // offboard control status
        int err_code_preparation_step_;         // detailed error code for ACADO preparation step
        int err_code_feedback_step_;            // detailed error code for ACADO feedback step
        bool re_init_horizon_;                  // re-initialize the horizon
        bool grid_map_valid_;
        bool terrain_was_disabled_;
};

} // namespace fw_nmpc

#endif // NONLINEAR_MPC_H_
