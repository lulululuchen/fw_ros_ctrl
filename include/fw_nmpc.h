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

// ROS includes
#include <ros/ros.h>
#include <ros/console.h>

// ROS msg includes
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_msgs/GridMapInfo.h>

// Eigen / tf includes
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

// ACADO includes
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

/* ACADO definitions */
#define NX ACADO_NX		// Number of differential state variables
#define NU ACADO_NU		// Number of control inputs
#define NOD	ACADO_NOD	// Number of online data values
#define NY ACADO_NY 	// Number of measurements/references on nodes 0..N - 1
#define NYN	ACADO_NYN	// Number of measurements/references on node N
#define N	ACADO_N			// Number of intervals in the horizon

/* global variables used by the solver */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace fw_nmpc {

	/* indexing enumeration */

	enum IndexStates {
	 IDX_X_POS = 0,
	 IDX_X_GAMMA = 3,
	 IDX_X_XI,
	 IDX_X_PHI
	}; // states

	enum IndexControls {
	 IDX_U_GAMMA = 0,
	 IDX_U_PHI
	}; // controls

	enum IndexOnlineData {
	 IDX_OD_V = 0,
	 IDX_OD_W,
	 IDX_OD_B = 4,
	 IDX_OD_GAMMA_P = 7,
	 IDX_OD_CHI_P,
	 IDX_OD_T_LAT,
	 IDX_OD_T_LON,
	 IDX_OD_V_SINK,
	 IDX_OD_V_CLMB,
	 IDX_OD_DELTA_H,
	 IDX_OD_TERR_ORIG_N,
	 IDX_OD_TERR_ORIG_E,
	 IDX_OD_TERR_DATA
	}; // online data

/*
 * @brief fw_nmpc class
 *
 * class that implements fixed-wing nmpc
 */
class FwNmpc {

public:

	FwNmpc();

	/* callbacks */
	void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
	void gridMapCb(const grid_map_msgs::GridMap::ConstPtr& msg);
	void gridMapInfoCb(const grid_map_msgs::GridMapInfo::ConstPtr& msg);
	void localPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void localVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void windEstCb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);

	/* sets */


	/* gets */


private:

	/* node handles */
	ros::NodeHandle nmpc_;

	/* subscribers */
	ros::Subscriber imu_sub_;
	ros::Subscriber local_pos_sub_;
	ros::Subscriber local_vel_sub_;
	ros::Subscriber wind_est_sub_;
	ros::Subscriber grid_map_sub_;

	/* publishers */
	ros::Publisher att_sp_pub_;

	/* functions */

	// check for subscriptions
	void checkSubs();

	// initialization
	void initAcadoVars();
	void initHorizon();
	int initNmpc();

	// iteration step
	int nmpcIteration();

	// publishing encapsulation
	void publishControls(uint64_t &t_ctrl, uint64_t t_iter_approx, int obctrl_status);
	void publishAcadoVars();
	void publishNmpcInfo(ros::Time t_iter_start, uint64_t t_ctrl, uint64_t t_solve, uint64_t t_update, uint64_t t_wp_man);

	// updates
	void updateAcadoOd(); 		// update ACADO online data
	void updateAcadoW(); 			// update ACADO weighting matrices
	void updateAcadoX0(); 		// update ACADO state measurements
	void updateAcadoY(); 			// update ACADO references

	// helpers XXX: maybe move these somewhere else.
	double constrain(const double x, const double xmin, const double xmax);

	void shutdown();

	/* estimated states */
	Eigen::Vector3d x0_pos_; 		// local position (ned) [m]
  Eigen::Vector3d x0_vel_;		// local velocity (ned) [m]
  Eigen::Vector3d x0_euler_;	// attitude (euler angles - RPY) [rad]
	Eigen::Vector3d x0_wind_;		// wind estimate (ned) [m/s]

	/* calculated states */
	double airsp_;							// airspeed [m/s]

	/* solver matrices */
	Eigen::Matrix<double, ACADO_NX, 1> x0_; 									// measured states
	Eigen::Matrix<double, ACADO_NX, ACADO_N + 1> x_; 					// states
  Eigen::Matrix<double, ACADO_NU, ACADO_N> u_; 							// controls
  Eigen::Matrix<double, ACADO_NY, ACADO_N> y_; 							// references
  Eigen::Matrix<double, ACADO_NYN, 1> yN_; 									// end term references
  Eigen::Matrix<double, ACADO_NY, ACADO_NY> W_; 						// weights
  Eigen::Matrix<double, ACADO_NYN, ACADO_NYN> WN_; 					// end term weights
	Eigen::Matrix<double, IDX_OD_TERR_DATA, ACADO_N + 1> od_; // online data (excluding terrain data)

	/* timing */
	double loop_rate_;
	ros::Time t_last_ctrl_;
	double t_step_;

	/* controller switch */
	bool bModeChanged;
	int	last_ctrl_mode;
	int obctrl_en_;

	/* continuity */
	bool bYawReceived;
	float last_yaw_msg_;
};

} // namespace fw_nmpc

#endif
