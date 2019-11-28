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
#include <fw_ctrl/NmpcControls.h>
#include <fw_ctrl/NmpcInfo.h>
#include <fw_ctrl/NmpcMeasurements.h>
#include <fw_ctrl/NmpcObjNRef.h>
#include <fw_ctrl/NmpcObjRef.h>
#include <fw_ctrl/NmpcOnlineData.h>
#include <fw_ctrl/NmpcStates.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <mavros_msgs::ActuatorControl.h>
#include <mavros_msgs::HomePosition.h>
#include <mavros_msgs::State.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

// Grid map
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>

// tf
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

// Eigen
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

// ACADO
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define NX ACADO_NX		// Number of differential state variables
#define NU ACADO_NU		// Number of control inputs
#define NOD	ACADO_NOD	// Number of online data values
#define NY ACADO_NY 	// Number of measurements/references on nodes 0..N - 1
#define NYN	ACADO_NYN	// Number of measurements/references on node N
#define N	ACADO_N			// Number of intervals in the horizon
#define NOCC 6				// Number of stored occlusion attributes

#define LEN_SLIDING_WINDOW_MAX ACADO_N 	// maximum length of sliding window
#define MIN_EXPONENTIAL_COST 1.0e-5 		// minimum exponential cost at input arg = 1 (for soft constraints)

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

#define DEG_TO_RAD 0.017453292519943 		// degrees to radians XXX: is there really no other function already built in for this?

#define AIR_GAS_CONST 287.1; 					 	// [J/(kg*K)]
#define ABSOLUTE_NULL_CELSIUS -273.15; 	// [C]
#define ONE_G 9.81											// [m/s^2]

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
} // outputs

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
} // radial occlusion priorities

/*
 * @brief fw_nmpc class
 *
 * class that implements fixed-wing nmpc
 */
class FwNmpc {

public:

    FwNmpc();

    /* callbacks */
    void actCb(const mavros_msgs::ActuatorControl::ConstPtr& msg);
    void gridMapCb(const grid_map_msgs::GridMap::ConstPtr& msg);
    void homePosCb(const mavros_msgs::HomePosition::ConstPtr& msg);
    void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
    void localPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void localVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void staticPresCb(const sensor_msgs::FluidPressure::ConstPtr& msg);
    void sysStatusCb(const mavros_msgs::State::ConstPtr& msg);
    void tempCCb(const sensor_msgs::Temperature::ConstPtr& msg);
    void windEstCb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);

private:

    /* node handles */
    ros::NodeHandle nmpc_;

    /* subscribers */
    ros::Subscriber act_sub_;
    ros::Subscriber grid_map_sub_;
    ros::Subscriber home_pos_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber local_pos_sub_;
    ros::Subscriber local_vel_sub_;
ros:
    Subscriber static_pres_sub_
    ros::Subscriber sys_status_sub_;
    ros::Subscriber temp_c_sub_;
    ros::Subscriber wind_est_sub_;

    /* publishers */
    ros::Publisher att_sp_pub_;
    ros::Publisher nmpc_info_pub_;
    ros::Publisher nmpc_meas_pub_;
    ros::Publisher nmpc_states_pub_;
    ros::Publisher nmpc_controls_pub_;
    ros::Publisher nmpc_online_data_pub_;
    ros::Publisher nmpc_obj_ref_pub_;
    ros::Publisher nmpc_objN_ref_pub_;
    ros::Publisher obctrl_status_pub_;
    ros::Publisher thrust_pub_;

    /* functions */

    // check for subscriptions
    void checkSubs();

    // helper functions
    double calc_air_density();
    double flaps_to_rad(const double flaps_normalized);
    double map_prop_speed_to_throt(const double n_prop, const double airsp, const double aoa);
    double map_px4_to_throt(const double px4_throt, const double airsp, const double aoa);
    double map_throt_to_prop_speed(const double throt, const double airsp, const double aoa);
    double map_throt_to_px4(const double throt, const double airsp, const double aoa);
    void propagate_virt_prop_speed(const double throt, const double airsp, const double aoa);
    double unwrap_heading(double yaw);

    // publishers
    void publishControls(double u_T, double phi_ref, double theta_ref);
    void publishNmpcInfo(ros::Time t_iter_start, uint64_t t_ctrl, uint64_t t_ext_obj, uint64_t t_solve, uint64_t t_update);
    void publishNmpcStates();

    // objective functions
    void preEvaluateObjectives();
    void updateObjectiveParameters();
    void shiftOcclusionSlidingWindow();
    void castRays(const double *terrain_data);
    void sumOcclusionDetections();
    void evaluateExternalObjectives(const double *terrain_data);
    void prioritizeObjectives();
    void filterControlReference();
    void filterTerrainCostJacobian();

    // NMPC functions
    void applyControl(); 						// apply current optimal control
    int nmpcIteration(); 						// nmpc iteration loop
    void updateAcadoConstraints(); 	// update ACADO constraints
    void updateAcadoOD(); 					// update ACADO online data
    void updateAcadoW(); 						// update ACADO weighting matrices
    void updateAcadoX0(); 					// update ACADO state measurements
    void updateAcadoY(); 						// update ACADO references

    // initialization
    void initAcadoVars();
    void initHorizon();
    int initNMPC();

    // ros node functions
    void shutdown();

    /* pixhawk states / estimates */
    double flaps_normalized_;		// normalized flaps setting
    double home_lat_;						// home position latitude [deg]
    double home_lon_;						// home position longitude [deg]
    double home_alt_;						// home position altitude (absolute) [m]
    bool offboard_mode_;				// system is in offboard control mode
    double px4_throt_;					// throttle from pixhawk
    double static_pres_;				// static pressure [Pa]
    double temp_c_;								// temperature [C]
    Eigen::Vector3d x0_pos_; 		// local position (ned) [m]
    Eigen::Vector3d x0_vel_;		// local velocity (ned) [m]
    Eigen::Vector3d x0_euler_;	// attitude (euler angles - RPY) [rad]
    Eigen::Vector3d x0_wind_;		// wind estimate (ned) [m/s]

    /* virtual states */
    double n_prop_virt_; 				// virtual propeller speed state [rps]
    bool first_yaw_received_;		// a yaw estimate was received (used for yaw unwrapping) [rad]
    float last_yaw_msg_;				// the last yaw estimate (used for yaw unwrapping) [rad]

    /* grid map */
    grid_map::GridMap terrain_map_;
    double terr_local_origin_n_;
    double terr_local_origin_e_;
    int map_height_;
    int map_width_;
    double map_resolution_;

    /* solver matrices */
    Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>> x0_; 						// measured states
    Eigen::Map<Eigen::Matrix<double, ACADO_NU, 1>> u_; 							// currently applied controls
    Eigen::Map<Eigen::Matrix<double, ACADO_NU, ACADO_N>> u_ref_; 		// control reference horizon
    Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>> y_; 				// objective references
    Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>> yN_; 						// end term objective references
    Eigen::Map<Eigen::Matrix<double, ACADO_NY, 1>> inv_y_scale_sq_; // inverse output scale squared diagonal
    Eigen::Map<Eigen::Matrix<double, ACADO_NY, 1>> w_; 							// weight diagonal
    Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N>> od_; 			// online data
    Eigen::Map<Eigen::Matrix<double, ACADO_NU, 1> > lb_; 						// lower bound of control constraints
    Eigen::Map<Eigen::Matrix<double, ACADO_NU, 1> > ub_; 				 		// upper bound of control constraints

    /* sliding window of occlusions */
    int len_sliding_window_;
    int occ_detect_slw[ACADO_N+LEN_SLIDING_WINDOW_MAX]; 			// sliding window of occlusion detections (row-major format)
    double occ_slw[(ACADO_N+LEN_SLIDING_WINDOW_MAX) * N_OCC]; // sliding window of occlusion attributes (row-major format)

    /* parameters */
    double aoa_params_[5];					// soft angle of attack parameters
    double terrain_params_[10];			// soft terrain parameters
    double log_sqrt_w_over_sig1_r_; // XXX: this is the only one currently needed outside the lsq_objective.c file..

    /* path */
    double path_error_lat_; 		// lateral-directional path following error
    double path_error_lon_; 		// longitudinal path following error
    double path_reference_[5]; 	// path reference parameters
    int path_type_; 						// 0 = constant altitude loiter, 1 = line

    /* objective priorities */
    Eigen::Map<Eigen::Matrix<double, ACADO_N+1, 1>> prio_aoa_; 	// soft angle of attack priority
    Eigen::Map<Eigen::Matrix<double, ACADO_N+1, 1>> prio_h_;		// soft nadir terrain priority
    Eigen::Map<Eigen::Matrix<double, ACADO_N+1, 4>> prio_r_;		// soft radial terrain priority

    /* timing */
    double loop_rate_;			// nmpc iteration loop rate [Hz]
    ros::Time t_last_ctrl_; // time since last control action [s]
    double t_step_;					// nmpc model time discretization
};

} // namespace fw_nmpc

#endif
