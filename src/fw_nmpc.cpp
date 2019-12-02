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

#include <ros/ros.h>
#include <ros/package.h>

#include <fw_nmpc/fw_nmpc.h>

#include <math.h>
#include <string.h>

using namespace fw_nmpc;

FwNMPC::FwNMPC() :
    flaps_normalized_(0.0),
    home_lat_(0),
    home_lon_(0),
    home_alt_(0),
    offboard_mode_(false),
    px4_throt_(0.0),
    static_pres_(101325.0),
    temp_c_(25.0),
    x0_pos_(Eigen::Vector3d::Zero()),
    x0_vel_(Eigen::Vector3d::Zero()),
    x0_euler_(Eigen::Vector3d::Zero()),
    x0_wind_(Eigen::Vector3d::Zero()),
    n_prop_virt_(0.0),
    first_yaw_received_(false),
    last_yaw_msg_(0.0),
    terr_local_origin_n_(0.0),
    terr_local_origin_e_(0.0),
    map_height_(100),
    map_width_(100),
    map_resolution_(5.0),
    x0_(Eigen::Matrix<double, ACADO_NX, 1>::Zero()),
    u_(Eigen::Matrix<double, ACADO_NU, 1>::Zero()),
    u_ref_(Eigen::Matrix<double, ACADO_NU, ACADO_N>::Zero()),
    y_(Eigen::Matrix<double, ACADO_NY, ACADO_N>::Zero()),
    yN_(Eigen::Matrix<double, ACADO_NYN, 1>::Zero()),
    inv_y_scale_sq_(Eigen::Matrix<double, ACADO_NY, 1>::Zero()),
    w_(Eigen::Matrix<double, ACADO_NY, 1>::Zero()),
    od_(Eigen::Matrix<double, ACADO_NOD, ACADO_N+1>::Zero()),
    lb_(Eigen::Matrix<double, ACADO_NU, 1>::Zero()),
    ub_(Eigen::Matrix<double, ACADO_NU, 1>::Zero()),
    len_sliding_window_(10),
    log_sqrt_w_over_sig1_r_(1.0),
    one_over_sqrt_w_r_(1.0),
    path_error_lat_(0.0),
    path_error_lon_(0.0),
    path_type_(0),
    prio_aoa_(Eigen::Matrix<double, ACADO_N+1, 1>::Ones()),
    prio_h_(Eigen::Matrix<double, ACADO_N+1, 1>::Ones()),
    prio_r_(Eigen::Matrix<double, ACADO_N+1, 4>::Ones()),
    obctrl_status_(0),
    re_init_horizon_(false)
{
    ROS_INFO("Instance of NMPC created");

    /* subscribers */
    act_sub_ = nmpc_.subscribe("/mavros/target_actuator_control", 1, &FwNMPC::actCb, this);
    grid_map_sub_ = nmpc_.subscribe("/elevation_mapper/map_mpc", 1, &FwNMPC::gridMapCb, this);
    home_pos_sub_ = nmpc_.subscribe("/mavros/home_position", 1, &FwNMPC::homePosCb, this);
    imu_sub_ = nmpc_.subscribe("/mavros/imu/data", 1, &FwNMPC::imuCb, this);
    local_pos_sub_ = nmpc_.subscribe("/mavros/local_position/pose", 1, &FwNMPC::localPosCb, this);
    local_vel_sub_ = nmpc_.subscribe("/mavros/local_position/velocity", 1, &FwNMPC::localVelCb, this);
    static_pres_sub_ = nmpc_.subscribe("/mavros/static_pressure", 1, &FwNMPC::staticPresCb, this);
    sys_status_sub_ = nmpc_.subscribe("/mavros/state", 1, &FwNMPC::sysStatusCb, this);
    temp_c_sub_ = nmpc_.subscribe("/mavros/temperature_baro", 1, &FwNMPC::tempCCb, this);
    wind_est_sub_ = nmpc_.subscribe("/mavros/wind_estimation", 1, &FwNMPC::windEstCb, this);

    /* publishers */
    att_sp_pub_ = nmpc_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1);
    nmpc_info_pub_ = nmpc_.advertise<fw_ctrl::NMPCInfo>("/nmpc/info",10,true);
    nmpc_meas_pub_  = nmpc_.advertise<fw_ctrl::NMPCMeasurements>("/nmpc/measurements",10,true);
    nmpc_states_pub_  = nmpc_.advertise<fw_ctrl::NMPCStates>("/nmpc/states",10,true);
    nmpc_controls_pub_  = nmpc_.advertise<fw_ctrl::NMPCControls>("/nmpc/controls",10,true);
    nmpc_online_data_pub_  = nmpc_.advertise<fw_ctrl::NMPCOnlineData>("/nmpc/online_data",10,true);
    nmpc_obj_ref_pub_  = nmpc_.advertise<fw_ctrl::NMPCObjRef>("/nmpc/obj_ref",10,true);
    nmpc_objN_ref_pub_  = nmpc_.advertise<fw_ctrl::NMPCObjNRef>("/nmpc/objN_ref",10,true);
    obctrl_status_pub_ = nmpc_.advertise<std_msgs::Int32>("/nmpc/status", 1);
    thrust_pub_ = nmpc_.advertise<std_msgs::Float32>("/mavros/setpoint_attitude/thrust", 1);
}

/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/
/* CALLBACKS / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

void FwNMPC::actCb(const mavros_msgs::ActuatorControl::ConstPtr& msg) // actuator control target msg callback
{
    if (msg->group_mix == msg->PX4_MIX_FLIGHT_CONTROL) {
        px4_throt_ = (double)msg->controls[3];
        flaps_normalized_ = (double)msg->controls[4];
    }
} // actCb

void FwNMPC::gridMapCb(const grid_map_msgs::GridMap& msg) // grid map msg callback
{
    bool ret;

    // get incoming terrain data
    ret = grid_map::GridMapRosConverter::fromMessage(msg, terrain_map_);
    if (ret) ROS_ERROR("grid map cb: failed to convert msg to eigen");

    // grid map center (assuming constant level-northing orientation in world frame)
    double map_center_north = msg.info.pose.position.x; // N (inertial frame) = x (grid map frame) [m]
    double map_center_east = -msg.info.pose.position.y; // E (inertial frame) = -y (grid map frame) [m]

    // get local terrain map origin (top-right corner of array)
    terr_local_origin_n_ = map_center_north - msg.info.length_x / 2.0;
    terr_local_origin_e_ = map_center_east - msg.info.length_y / 2.0;

    // map dimensions
    map_resolution_ = msg.info.resolution;
    map_height_ = msg.info.length_x / map_resolution_; //XXX: CHECK THIS.. not sure if length is res*Ncells, or res*Ncells-1
    map_width_ = msg.info.length_y / map_resolution_;

    // convert local terrain map to row major array // TODO: should be a better way than needing to create these and pass as pointer every time.. e.g. pass the eigen matrix or gridmap itself
    terrain_map_["elevation"].colwise().reverse(); // TODO: sync the format of the acado functions with grid map.. or use grid map in the solver functions themselves
    if (map_height_*map_width_ > MAX_SIZE_TERR_ARRAY) {
        ROS_ERROR("grid map cb: received terrain map exceeds maximum allowed size");
    }
    else {
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>(terr_array_, map_height_, map_width_) = terrain_map_["elevation"].cast<double>();
    }
} // gridMapCb

void FwNMPC::homePosCb(const mavros_msgs::HomePosition::ConstPtr& msg) // home position msg callback
{
    home_lat_ = (double)msg->geo.latitude;
    home_lon_ = (double)msg->geo.longitude;
    home_alt_ = (double)msg->geo.altitude;
} // homePosCb

void FwNMPC::imuCb(const sensor_msgs::Imu::ConstPtr& msg) // imu msg callback
{
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(msg->orientation, q);
    x0_euler_ = q.toRotationMatrix().eulerAngles(0, 1, 2);
} // imuCb

void FwNMPC::localPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg) // local position msg callback
{
    // ENU -> NED
    x0_pos_(0) = msg->pose.position.y;
    x0_pos_(1) = msg->pose.position.x;
    x0_pos_(2) = -msg->pose.position.z;
} // localPosCb

void FwNMPC::localVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg) // local velocity msg callback
{
    // ENU -> NED
    x0_vel_(0) = msg->twist.linear.y;
    x0_vel_(1) = msg->twist.linear.x;
    x0_vel_(2) = -msg->twist.linear.z;
} // localVelCb

void FwNMPC::staticPresCb(const sensor_msgs::FluidPressure::ConstPtr& msg) // static pressure msg callback
{
    static_pres_ = msg->fluid_pressure;
} // staticPresCb

void FwNMPC::sysStatusCb(const mavros_msgs::State::ConstPtr& msg) // system status msg callback
{
    // this message comes at 1 Hz and resets the NMPC if not in OFFBOARD mode
    ROS_INFO_STREAM("Received Pixhawk Mode: " <<  msg->mode);
    offboard_mode_ = msg->mode == "OFFBOARD";
} // sysStatusCb

void FwNMPC::tempCCb(const sensor_msgs::Temperature::ConstPtr& msg) // temperature msg callback
{
    temp_c_ = msg->temperature;
} // tempCCb

void FwNMPC::windEstCb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg) // wind estimation msg callback
{
    // ACTUALLY one that is in NED already
    tf::vectorMsgToEigen(msg->twist.twist.linear, x0_wind_);
} // windEstCb

void FwNMPC::checkSubs()
{
    /* wait for home waypoint */
    while (home_lat_ < 0.1 && home_lon_ < 0.1) {
        ros::spinOnce();
        ROS_INFO ("check subs: waiting for home position");
        sleep(1.0);
    }
    ROS_ERROR("check subs: received home position");

    // XXX: should have checks (and/or timeouts) for the rest of the required state estimates

    return;
} // checkSubs

/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/
/* GETS  / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

double FwNMPC::getLoopRate()
{
    double loop_rate;
    nmpc_.getParam("/nmpc/loop_rate", loop_rate);   // iteration loop rate [Hz]

    return loop_rate;
} // getLoopRate

double FwNMPC::getTimeStep()
{
    double t_step;
    nmpc_.getParam("/nmpc/time_step", t_step);      // get model discretization step [s]

    return t_step;
} // getTimeStep

/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/
/* MATH FUNCTIONS  / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

int FwNMPC::constrain_int(int x, int xmin, int xmax) {
    return (x < xmin) ? xmin : ((x > xmax) ? xmax : x);
} // constrain_int

double FwNMPC::constrain_double(double x, double xmin, double xmax)
{
    return (x < xmin) ? xmin : ((x > xmax) ? xmax : x);
} // constrain_double

void FwNMPC::cross(double *v, const double v1[3], const double v2[3])
{
    v[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v[2] = v1[0]*v2[1] - v1[1]*v2[0];
} // cross

double FwNMPC::dot(const double v1[3], const double v2[3])
{
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
} // dot

/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/
/* HELPER FUNCTIONS  / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

double FwNMPC::calcAirDensity()
{
    return static_pres_ / (AIR_GAS_CONST * (temp_c_ - ABSOLUTE_NULL_CELSIUS));
} // calcAirDensity

// XXX: another duplicate from lsq_objective!
void FwNMPC::calculate_speed_states(double *speed_states,
        const double v, const double gamma, const double xi,
        const double w_n, const double w_e, const double w_d)
{
    const double v_cos_gamma = v*cos(gamma);
    const double cos_xi = cos(xi);
    const double sin_xi = sin(xi);

    /* airspeed */
    speed_states[0] = v_cos_gamma*cos_xi;       /* v_n */
    speed_states[1] = v_cos_gamma*sin_xi;       /* v_e */
    speed_states[2] = -v*sin(gamma);            /* v_d */

    /* ground speed */
    speed_states[3] = speed_states[0] + w_n;    /* vG_n */
    speed_states[4] = speed_states[1] + w_e;    /* vG_e */
    speed_states[5] = speed_states[2] + w_d;    /* vG_d */
    speed_states[6] = speed_states[3]*speed_states[3] + speed_states[4]*speed_states[4] + speed_states[5]*speed_states[5]; /* vG_sq */
    speed_states[7] = sqrt(speed_states[6]);    /* vG_norm */

    /* unit ground speed */
    speed_states[8] = (speed_states[7] < 0.01) ? 100.0 : 1.0 / speed_states[7];
    speed_states[9] = speed_states[3] * speed_states[8];    /* vG_n_unit */
    speed_states[10] = speed_states[4] * speed_states[8];	/* vG_e_unit */
    speed_states[11] = speed_states[5] * speed_states[8];   /* vG_d_unit */
}

double FwNMPC::flapsToRad(const double flaps_normalized)
{
    // for now, assuming that a full normalized signal corresponds to full actuation .. need to check this with px4 convention
    double flaps_lim;
    nmpc_.getParam("/nmpc/veh/flaps_lim", flaps_lim); // in deg
    flaps_lim *= DEG_TO_RAD;

    return flaps_lim * constrain_double(flaps_normalized, -1.0, 1.0);
} // flapsToRad

double FwNMPC::mapPropSpeedToThrot(const double n_prop, const double airsp, const double aoa)
{
    // prop speed input (converted from throttle input considering inflow and zero thrusting conditions)

    double vpmin, vpmax, n_T0_vmin, n_T0_vmax, n_max, epsilon_T;
    nmpc_.getParam("/nmpc/prop/vpmin", vpmin);           // minimum propeller inflow [m/s]
    nmpc_.getParam("/nmpc/prop/vpmax", vpmax);           // maximum propeller inflow [m/s]
    nmpc_.getParam("/nmpc/prop/n_T0_vmin", n_T0_vmin);   // zero-thrust prop speed at minimum airspeed [rps]
    nmpc_.getParam("/nmpc/prop/n_T0_vmax", n_T0_vmax);   // zero-thrust prop speed at maximum airspeed [rps]
    nmpc_.getParam("/nmpc/prop/n_max", n_max);           // maximum prop speed [rps]
    nmpc_.getParam("/nmpc/prop/epsilon_T", epsilon_T);   // thrust incidence [deg]
    epsilon_T *= DEG_TO_RAD;                             // thrust incidence [rad]

    const double vp = airsp * cos(aoa - epsilon_T); // inflow at propeller
    const double sig_vp = constrain_double((vp - vpmin) / (vpmax - vpmin), 0.0, 1.0); // prop inflow linear interpolater

    return (n_prop - n_T0_vmin * (1.0 - sig_vp) - n_T0_vmax * sig_vp)/(n_max - n_T0_vmin);
} // mapPropSpeedToThrot

double FwNMPC::mapPX4ToThrot(const double px4_throt, const double airsp, const double aoa)
{
    double n_0, n_max, n_delta;
    nmpc_.getParam("/nmpc/prop/n_0", n_0);               // prop speed at zero inflow and zero input [rps]
    nmpc_.getParam("/nmpc/prop/n_max", n_max);           // maximum prop speed [rps]
    nmpc_.getParam("/nmpc/prop/n_delta", n_delta);      // slope of first order prop speed to throttle input (PX4) mapping approximation

    return mapPropSpeedToThrot(px4_throt * n_delta + n_0, airsp, aoa);
} // mapPX4ToThrot

double FwNMPC::mapThrotToPropSpeed(const double throt, const double airsp, const double aoa)
{
    // prop speed input (converted from throttle input considering inflow and zero thrusting conditions)

    double vpmin, vpmax, n_T0_vmin, n_T0_vmax, n_max, epsilon_T;
    nmpc_.getParam("/nmpc/prop/vpmin", vpmin);           // minimum propeller inflow [m/s]
    nmpc_.getParam("/nmpc/prop/vpmax", vpmax);           // maximum propeller inflow [m/s]
    nmpc_.getParam("/nmpc/prop/n_T0_vmin", n_T0_vmin);   // zero-thrust prop speed at minimum airspeed [rps]
    nmpc_.getParam("/nmpc/prop/n_T0_vmax", n_T0_vmax);   // zero-thrust prop speed at maximum airspeed [rps]
    nmpc_.getParam("/nmpc/prop/n_max", n_max);           // maximum prop speed [rps]
    nmpc_.getParam("/nmpc/prop/epsilon_T", epsilon_T);   // thrust incidence [deg]
    epsilon_T *= DEG_TO_RAD;                            // thrust incidence [rad]

    const double vp = airsp * cos(aoa - epsilon_T); // inflow at propeller
    const double sig_vp = constrain_double((vp - vpmin) / (vpmax - vpmin), 0.0, 1.0); // prop inflow linear interpolater

    return (n_T0_vmin + throt * (n_max - n_T0_vmin)) * (1.0 - sig_vp) + (n_T0_vmax + throt * (n_max - n_T0_vmax)) * sig_vp;
} // mapThrotToPropSpeed

double FwNMPC::mapThrotToPX4(const double throt, const double airsp, const double aoa)
{
    double n_0, n_max, n_delta;
    nmpc_.getParam("/nmpc/prop/n_0", n_0);               // prop speed at zero inflow and zero input [rps]
    nmpc_.getParam("/nmpc/prop/n_max", n_max);           // maximum prop speed [rps]
    nmpc_.getParam("/nmpc/prop/n_delta", n_delta);      // slope of first order prop speed to throttle input (PX4) mapping approximation

    return (mapThrotToPropSpeed(throt, airsp, aoa) - n_0) / n_delta; // NOTE: this does not include dead-zone or true zero (prop may slowly spin at zero input)
} // mapThrotToPX4

void FwNMPC::propagateVirtPropSpeed(const double throt, const double airsp, const double aoa)
{
    double tau_n_prop;
    nmpc_.getParam("/nmpc/od/tau_n_prop", tau_n_prop);
    n_prop_virt_ = (mapThrotToPropSpeed(throt, airsp, aoa) - n_prop_virt_) / tau_n_prop / getLoopRate() + n_prop_virt_;
} // propagateVirtPropSpeed

double FwNMPC::unwrapHeading(double yaw)
{
    // XXX: once slip is included.. need to disambiguate heading (xi) vs yaw notation abuse here..

    if (!first_yaw_received_) {
        last_yaw_msg_ = yaw;
        first_yaw_received_ = true;
        return yaw;
    }

    float delta_yaw = yaw - last_yaw_msg_;
    if ( delta_yaw < -M_PI ) delta_yaw = delta_yaw + 2.0 * M_PI;
    if ( delta_yaw > M_PI ) delta_yaw = delta_yaw - 2.0 * M_PI;

    yaw = yaw + delta_yaw;

    last_yaw_msg_ = yaw;
    return yaw;
} // unwrapHeading

/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/
/* PUBLISHERS  / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

void FwNMPC::publishControls(const double u_T, const double phi_ref, const double theta_ref)
{
    // thrust sp
    std_msgs::Float32 thrust_sp;
    thrust_sp.data = constrain_double(mapThrotToPX4(u_T, acadoVariables.x[IDX_X_V], acadoVariables.x[IDX_X_THETA] - acadoVariables.x[IDX_X_GAMMA]), 0.0, 1.0);

    thrust_pub_.publish(thrust_sp);

    // attitude setpoint
    geometry_msgs::PoseStamped att_sp;
    att_sp.header.frame_id = "world";
    Eigen::AngleAxisd rollAngle(phi_ref, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(theta_ref, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond att_sp_quat_eigen = yawAngle * pitchAngle * rollAngle;
    tf::quaternionEigenToMsg(att_sp_quat_eigen, att_sp.pose.orientation);

    att_sp_pub_.publish(att_sp);
} // publishControls

void FwNMPC::publishNMPCInfo(ros::Time t_iter_start, uint64_t t_ctrl, uint64_t t_preeval, uint64_t t_prep, uint64_t t_fb)
{
    fw_ctrl::NMPCInfo nmpc_info;

    // obctrl status
    nmpc_info.status = obctrl_status_;

    /* solver info */
    nmpc_info.kkt = (double)acado_getKKT();
    nmpc_info.obj = (double)acado_getObjective(); // NOTE: this value will be based on the linearizations from the previous optimized states (objectives not re-pre-evaluated)

    // various elapsed times
    nmpc_info.t_ctrl = t_ctrl;          // time elapsed since last control action was published (stays zero if not in auto mode)
    nmpc_info.t_preeval = t_preeval;    // time elapsed during objective pre-evaluation (includes array updates etc)
    nmpc_info.t_prep = t_prep;          // time elapsed during nmpc preparation step
    nmpc_info.t_fb = t_fb;              // time elapsed during nmpc feedback step

    // nmpc iteration time in us
    ros::Duration t_elapsed = ros::Time::now() - t_iter_start;
    nmpc_info.t_iter = (uint64_t)(t_elapsed.toNSec()/1000);

    nmpc_info_pub_.publish(nmpc_info);
} // publishNMPCInfo

void FwNMPC::publishNMPCStates()
{
    fw_ctrl::NMPCMeasurements nmpc_measurements;
    fw_ctrl::NMPCStates nmpc_states;
    fw_ctrl::NMPCControls nmpc_controls;
    fw_ctrl::NMPCOnlineData nmpc_online_data;
    fw_ctrl::NMPCObjRef nmpc_obj_ref;
    fw_ctrl::NMPCObjNRef nmpc_objN_ref;

    /* measurements */
    nmpc_measurements.n  = (float)x0_(0);
    nmpc_measurements.e  = (float)x0_(1);
    nmpc_measurements.d  = (float)x0_(2);
    nmpc_measurements.v  = (float)x0_(3);
    nmpc_measurements.gamma  = (float)x0_(4);
    nmpc_measurements.xi  = (float)x0_(5);
    nmpc_measurements.phi  = (float)x0_(6);
    nmpc_measurements.theta  = (float)x0_(7);
    nmpc_measurements.n_prop  = (float)x0_(8);

    /* state/control horizons */
    for (int i=0; i<N; i++) {
        nmpc_states.n[i]  = (float)acadoVariables.x[NX * i + IDX_X_POS];
        nmpc_states.e[i]  = (float)acadoVariables.x[NX * i + IDX_X_POS+1];
        nmpc_states.d[i]  = (float)acadoVariables.x[NX * i + IDX_X_POS+2];
        nmpc_states.v[i]  = (float)acadoVariables.x[NX * i + IDX_X_V];
        nmpc_states.gamma[i]  = (float)acadoVariables.x[NX * i + IDX_X_GAMMA];
        nmpc_states.xi[i]  = (float)acadoVariables.x[NX * i + IDX_X_XI];
        nmpc_states.phi[i]  = (float)acadoVariables.x[NX * i + IDX_X_PHI];
        nmpc_states.theta[i]  = (float)acadoVariables.x[NX * i + IDX_X_THETA];
        nmpc_states.n_prop[i]  = (float)acadoVariables.x[NX * i + IDX_X_NPROP];

        nmpc_controls.u_T[i] = (float)acadoVariables.u[NU * i + IDX_U_U_T];
        nmpc_controls.phi_ref[i] = (float)acadoVariables.u[NU * i + IDX_U_PHI_REF];
        nmpc_controls.theta_ref[i] = (float)acadoVariables.u[NU * i + IDX_U_THETA_REF];
    }
    nmpc_states.n[N]  = (float)acadoVariables.x[NX * N + IDX_X_POS];
    nmpc_states.e[N]  = (float)acadoVariables.x[NX * N + IDX_X_POS+1];
    nmpc_states.d[N]  = (float)acadoVariables.x[NX * N + IDX_X_POS+2];
    nmpc_states.v[N]  = (float)acadoVariables.x[NX * N + IDX_X_V];
    nmpc_states.gamma[N]  = (float)acadoVariables.x[NX * N + IDX_X_GAMMA];
    nmpc_states.xi[N]  = (float)acadoVariables.x[NX * N + IDX_X_XI];
    nmpc_states.phi[N]  = (float)acadoVariables.x[NX * N + IDX_X_PHI];
    nmpc_states.theta[N]  = (float)acadoVariables.x[NX * N + IDX_X_THETA];
    nmpc_states.n_prop[N]  = (float)acadoVariables.x[NX * N + IDX_X_NPROP];

    /* online data */
    nmpc_online_data.rho = (float)acadoVariables.od[IDX_OD_RHO];
    nmpc_online_data.wn = (float)acadoVariables.od[IDX_OD_W];
    nmpc_online_data.we = (float)acadoVariables.od[IDX_OD_W+1];
    nmpc_online_data.wd = (float)acadoVariables.od[IDX_OD_W+2];
    nmpc_online_data.tau_phi = (float)acadoVariables.od[IDX_OD_TAU_PHI];
    nmpc_online_data.tau_theta = (float)acadoVariables.od[IDX_OD_TAU_THETA];
    nmpc_online_data.k_phi = (float)acadoVariables.od[IDX_OD_K_PHI];
    nmpc_online_data.k_theta = (float)acadoVariables.od[IDX_OD_K_THETA];
    nmpc_online_data.tau_n = (float)acadoVariables.od[IDX_OD_TAU_N];
    nmpc_online_data.delta_F = (float)acadoVariables.od[IDX_OD_DELTA_F];
    for (int i=0; i<N+1; i++) { // all prior online data are held constant through the horizon
        nmpc_online_data.soft_aoa[i] = (float)acadoVariables.od[NOD * i + IDX_OD_SOFT_AOA];
        nmpc_online_data.jac_soft_aoa_0[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_AOA];
        nmpc_online_data.jac_soft_aoa_1[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_AOA+1];
        nmpc_online_data.soft_h[i] = (float)acadoVariables.od[NOD * i + IDX_OD_SOFT_H];
        nmpc_online_data.jac_soft_h_0[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_H];
        nmpc_online_data.jac_soft_h_1[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_H+1];
        nmpc_online_data.jac_soft_h_2[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_H+2];
        nmpc_online_data.jac_soft_h_3[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_H+3];
        nmpc_online_data.soft_r[i] = (float)acadoVariables.od[NOD * i + IDX_OD_SOFT_R];
        nmpc_online_data.jac_soft_r_0[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R];
        nmpc_online_data.jac_soft_r_1[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R+1];
        nmpc_online_data.jac_soft_r_2[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R+2];
        nmpc_online_data.jac_soft_r_3[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R+3];
        nmpc_online_data.jac_soft_r_4[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R+4];
        nmpc_online_data.jac_soft_r_5[i] = (float)acadoVariables.od[NOD * i + IDX_OD_JAC_SOFT_R+5];
    }

    /* objective references */
    for (int i=0; i<N; i++) {
        nmpc_obj_ref.vn[i] = (float)acadoVariables.y[NY * i + IDX_Y_VN];
        nmpc_obj_ref.ve[i] = (float)acadoVariables.y[NY * i + IDX_Y_VE];
        nmpc_obj_ref.vd[i] = (float)acadoVariables.y[NY * i + IDX_Y_VD];
        nmpc_obj_ref.v[i] = (float)acadoVariables.y[NY * i + IDX_Y_V];
        nmpc_obj_ref.phi[i] = (float)acadoVariables.y[NY * i + IDX_Y_PHI];
        nmpc_obj_ref.theta[i] = (float)acadoVariables.y[NY * i + IDX_Y_THETA];
        nmpc_obj_ref.soft_aoa[i] = (float)acadoVariables.y[NY * i + IDX_Y_SOFT_AOA];
        nmpc_obj_ref.soft_h[i] = (float)acadoVariables.y[NY * i + IDX_Y_SOFT_H];
        nmpc_obj_ref.soft_r[i] = (float)acadoVariables.y[NY * i + IDX_Y_SOFT_R];

        nmpc_obj_ref.u_T[i] = (float)acadoVariables.y[NY * i + IDX_Y_U_T];
        nmpc_obj_ref.phi_ref[i] = (float)acadoVariables.y[NY * i + IDX_Y_PHI_REF];
        nmpc_obj_ref.theta_ref[i] = (float)acadoVariables.y[NY * i + IDX_Y_THETA_REF];
    }
    nmpc_objN_ref.vn = (float)acadoVariables.yN[IDX_Y_VN];
    nmpc_objN_ref.ve = (float)acadoVariables.yN[IDX_Y_VE];
    nmpc_objN_ref.vd = (float)acadoVariables.yN[IDX_Y_VD];
    nmpc_objN_ref.v = (float)acadoVariables.yN[IDX_Y_V];
    nmpc_objN_ref.phi = (float)acadoVariables.yN[IDX_Y_PHI];
    nmpc_objN_ref.theta = (float)acadoVariables.yN[IDX_Y_THETA];
    nmpc_objN_ref.soft_aoa = (float)acadoVariables.yN[IDX_Y_SOFT_AOA];
    nmpc_objN_ref.soft_h = (float)acadoVariables.yN[IDX_Y_SOFT_H];
    nmpc_objN_ref.soft_r = (float)acadoVariables.yN[IDX_Y_SOFT_R];

    // publish
    nmpc_meas_pub_.publish(nmpc_measurements);
    nmpc_states_pub_.publish(nmpc_states);
    nmpc_controls_pub_.publish(nmpc_controls);
    nmpc_online_data_pub_.publish(nmpc_online_data);
    nmpc_obj_ref_pub_.publish(nmpc_obj_ref);
    nmpc_objN_ref_pub_.publish(nmpc_objN_ref);
} // publishNMPCStates

/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/
/* OBJECTIVE PRE-EVALUATION FUNCTIONS  / / / / / / / / / / / / / / / / / / / /*/
/*
 * Collection of functions used for (pre)evaluating MPC objectives outside the
 * ACADO model.
 */

void FwNMPC::preEvaluateObjectives()
{
    // TODO: organize this whole thing better..

    /* update params */
    updateObjectiveParameters();

    /* sliding window operations */
    shiftOcclusionSlidingWindow();
    castRays(terr_array_); //XXX: dont really need to pass this, as the variable is global within the class

    /* external objectives */
    sumOcclusionDetections();
    evaluateExternalObjectives(terr_array_);
} // preEvaluateObjectives

void FwNMPC::updateObjectiveParameters()
{
    // TODO: is there a way to only run through this when we know something is updated? e.g. especially with the "log" and "division" operations in the soft constraint params..
    // XXX: THIS FUNCTION IS DISGUSTING -> CLEAN THIS UP!

    // FOR the weight-dependent cost shaping params: log_sqrt_w_over_sig1_XXX, one_over_sqrt_w_XXX
    // NOTE: depends on the current (un-modulated) weighting - evaluate PRIOR to updating current weighting priorities
    // NOTE: if soft constraints become dependent on weighting of other soft constraints, this will need some re-working

    // sliding occlusion window
    nmpc_.getParam("/nmpc/len_slw", len_sliding_window_);
    len_sliding_window_ = constrain_int(len_sliding_window_, 1, LEN_SLIDING_WINDOW_MAX);

    // soft angle of attack parameters
    nmpc_.getParam("/nmpc/aoa_params/delta_aoa", aoa_params_[0]);
    aoa_params_[0] *= DEG_TO_RAD;                                           // angle of attack buffer [rad]
    nmpc_.getParam("/nmpc/aoa_params/aoa_m", aoa_params_[1]);
    aoa_params_[1] *= DEG_TO_RAD;                                           // angle of attack lower bound [rad]
    nmpc_.getParam("/nmpc/aoa_params/aoa_p", aoa_params_[2]);
    aoa_params_[2] *= DEG_TO_RAD;                                           // angle of attack upper bound [rad]

    double sig_aoa_1, log_sqrt_w_over_sig1_aoa, one_over_sqrt_w_aoa;
    nmpc_.getParam("/nmpc/aoa_params/sig_aoa_1", sig_aoa_1);
    if (sig_aoa_1 < MIN_EXPONENTIAL_COST) sig_aoa_1 = MIN_EXPONENTIAL_COST; // angle of attack minimum soft exponential cost at unit height = 1

    if (acadoVariables.W[NY * IDX_Y_SOFT_AOA + IDX_Y_SOFT_AOA] <= sig_aoa_1) {
        // disable
        log_sqrt_w_over_sig1_aoa = 0.0;
        one_over_sqrt_w_aoa = -1.0;
    }
    else {
        log_sqrt_w_over_sig1_aoa = log(sqrt(acadoVariables.W[NY * IDX_Y_SOFT_AOA + IDX_Y_SOFT_AOA] / sig_aoa_1));
        one_over_sqrt_w_aoa = 1.0 / sqrt(acadoVariables.W[NY * IDX_Y_SOFT_AOA + IDX_Y_SOFT_AOA]);
    }

    // soft nadir terrain parameters
    nmpc_.getParam("/nmpc/terrain_params/h_offset", terrain_params_[0]);   // nadir terrain offset [m]
    nmpc_.getParam("/nmpc/terrain_params/delta_h", terrain_params_[1]);   // nadir terrain buffer [m]

    double sig_h_1, log_sqrt_w_over_sig1_h, one_over_sqrt_w_h;
    nmpc_.getParam("/nmpc/terrain_params/sig_h_1", sig_h_1);
    if (sig_h_1 < MIN_EXPONENTIAL_COST) sig_h_1 = MIN_EXPONENTIAL_COST;   // nadir terrain minimum soft exponential cost at unit height = 1

    if (acadoVariables.W[NY * IDX_Y_SOFT_H + IDX_Y_SOFT_H] <= sig_h_1) {
        // disable
        log_sqrt_w_over_sig1_h = 0.0;
        one_over_sqrt_w_h = -1.0;
    }
    else {
        log_sqrt_w_over_sig1_h = log(sqrt(acadoVariables.W[NY * IDX_Y_SOFT_H + IDX_Y_SOFT_H] / sig_h_1));
        one_over_sqrt_w_h = 1.0 / sqrt(acadoVariables.W[NY * IDX_Y_SOFT_H + IDX_Y_SOFT_H]);
    }

    // soft radial terrain parameters
    nmpc_.getParam("/nmpc/terrain_params/r_offset", terrain_params_[4]);   // radial terrain offset [m]
    nmpc_.getParam("/nmpc/terrain_params/delta_r0", terrain_params_[5]);   // radial terrain buffer (at zero relative velocity) [m]

    double k_r_offset, k_delta_r;
    nmpc_.getParam("/nmpc/terrain_params/k_r_offset", k_r_offset);         // gain on relative velocity term of radial offset
    nmpc_.getParam("/nmpc/terrain_params/k_delta_r", k_delta_r);           // gain on relative velocity term of radial buffer

    double sig_r_1, log_sqrt_w_over_sig1_r, one_over_sqrt_w_r;
    nmpc_.getParam("/nmpc/terrain_params/sig_r_1", sig_r_1);
    if (sig_r_1 < MIN_EXPONENTIAL_COST) sig_r_1 = MIN_EXPONENTIAL_COST;   // radial terrain minimum soft exponential cost at unit height = 1

    if (acadoVariables.W[NY * IDX_Y_SOFT_R + IDX_Y_SOFT_R] <= sig_r_1) {
        // disable
        log_sqrt_w_over_sig1_r = 0.0;
        one_over_sqrt_w_r = -1.0;
    }
    else {
        log_sqrt_w_over_sig1_r = log(sqrt(acadoVariables.W[NY * IDX_Y_SOFT_R + IDX_Y_SOFT_R] / sig_r_1));
        one_over_sqrt_w_r = 1.0 / sqrt(acadoVariables.W[NY * IDX_Y_SOFT_R + IDX_Y_SOFT_R]);
    }

    /* populate (remainder of) aoa param array (XXX: used for lsq obj functions.. may want to get rid of this struct or at least index it in the future) */
    aoa_params_[3] = log_sqrt_w_over_sig1_aoa;
    aoa_params_[4] = one_over_sqrt_w_aoa;

    /* populate (remainder of) terrain param array (XXX: used for lsq obj functions.. may want to get rid of this struct or at least index it in the future) */
    terrain_params_[2] = log_sqrt_w_over_sig1_h;
    terrain_params_[3] = one_over_sqrt_w_h;
    //XXX: need to introduce a roll_lim_ variable at some point which can deviate from the constraints (smaller)
    terrain_params_[6] = k_r_offset / tan(constrain_double(ub_(IDX_U_PHI_REF), 0.01, M_PI_2)) / ONE_G; // node-wise ground speed squared is multiplied in the internal ACADO lsq objective to make a multiple of the minimum turning radius
    terrain_params_[7] = k_delta_r / tan(constrain_double(ub_(IDX_U_PHI_REF), 0.01, M_PI_2)) / ONE_G; // node-wise ground speed squared is multiplied in the internal ACADO lsq objective to make a multiple of the minimum turning radius
    terrain_params_[8] = log_sqrt_w_over_sig1_r;
    terrain_params_[9] = one_over_sqrt_w_r;

    // XXX: these two are currently the only ones needed outside the lsq_objective.c file..
    log_sqrt_w_over_sig1_r_ = log_sqrt_w_over_sig1_r;
    one_over_sqrt_w_r_ = one_over_sqrt_w_r;

    /* guidance parameters */
    nmpc_.getParam("/nmpc/guidance/T_lat", guidance_params_[0]);
    nmpc_.getParam("/nmpc/guidance/T_lon", guidance_params_[1]);
    nmpc_.getParam("/nmpc/guidance/gamma_app_max", guidance_params_[2]);
    guidance_params_[2] *= DEG_TO_RAD;
    nmpc_.getParam("/nmpc/guidance/use_occ_as_guidance", guidance_params_[3]);


    /* path reference */
    nmpc_.getParam("/nmpc/path/path_type", path_type_);
    nmpc_.getParam("/nmpc/path/b_n", path_reference_[0]);
    nmpc_.getParam("/nmpc/path/b_e", path_reference_[1]);
    nmpc_.getParam("/nmpc/path/b_d", path_reference_[2]);
    nmpc_.getParam("/nmpc/path/Gamma_p", path_reference_[3]);
    path_reference_[3] *= DEG_TO_RAD;
    nmpc_.getParam("/nmpc/path/chi_p", path_reference_[4]);
    path_reference_[4] *= DEG_TO_RAD;

} // updateObjectiveParameters

void FwNMPC::shiftOcclusionSlidingWindow()
{
    /* shift detection window */
    for (int i = 0; i < len_sliding_window_; ++i) {
        occ_detect_slw_[i] = occ_detect_slw_[i+1];
        occ_slw_[i * NOCC + IDX_OCC_POS] = occ_slw_[(i+1) * NOCC + IDX_OCC_POS];
        occ_slw_[i * NOCC + IDX_OCC_POS+1] = occ_slw_[(i+1) * NOCC + IDX_OCC_POS+1];
        occ_slw_[i * NOCC + IDX_OCC_POS+2] = occ_slw_[(i+1) * NOCC + IDX_OCC_POS+2];
        occ_slw_[i * NOCC + IDX_OCC_NORMAL] = occ_slw_[(i+1) * NOCC + IDX_OCC_NORMAL];
        occ_slw_[i * NOCC + IDX_OCC_NORMAL+1] = occ_slw_[(i+1) * NOCC + IDX_OCC_NORMAL+1];
        occ_slw_[i * NOCC + IDX_OCC_NORMAL+2] = occ_slw_[(i+1) * NOCC + IDX_OCC_NORMAL+2];
    }
} // shiftOcclusionSlidingWindow

void FwNMPC::castRays(const double *terrain_data)
{
    /* cast rays along ground speed vector at every node */
    for (int i = 0; i < N+1; ++i)
    {
        /* calculate speed states */
        const double v = acadoVariables.x[i * NX + IDX_X_V];
        const double gamma = acadoVariables.x[i * NX + IDX_X_GAMMA];
        const double xi = acadoVariables.x[i * NX + IDX_X_XI];
        const double w_n = acadoVariables.od[i * NOD + IDX_OD_W];
        const double w_e = acadoVariables.od[i * NOD + IDX_OD_W+1];
        const double w_d = acadoVariables.od[i * NOD + IDX_OD_W+2];
        double speed_states[12];
        calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);

        /* forward ray */
        double p_occ_fwd[3];
        double n_occ_fwd[3];
        double r_occ_fwd; // XXX: not currently used
        int occ_detected_fwd = 0;
        get_occ_along_gsp_vec(p_occ_fwd, n_occ_fwd, &r_occ_fwd, &occ_detected_fwd,
                              acadoVariables.x + (i * ACADO_NX), speed_states, terrain_params_,
                              terr_local_origin_n_, terr_local_origin_e_, map_height_, map_width_,
                              map_resolution_, terrain_data);

        /* log detections */
        occ_detect_slw_[len_sliding_window_-1+i] = occ_detected_fwd;
        occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_POS] = p_occ_fwd[0];
        occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_POS+1] = p_occ_fwd[1];
        occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_POS+2] = p_occ_fwd[2];
        occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_NORMAL] = n_occ_fwd[0];
        occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_NORMAL+1] = n_occ_fwd[1];
        occ_slw_[(len_sliding_window_-1+i) * NOCC + IDX_OCC_NORMAL+2] = n_occ_fwd[2];
    }
} // castRays

void FwNMPC::sumOcclusionDetections()
{
    /* sum detections on sliding horizon window for each node */
    for (int i = 0; i < ACADO_N+1; ++i)
    {
        /* init */
        double jac_sig_r_fwd[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
        double jac_r_unit[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
        double r_unit_min;
        bool f_min;
        int occ_count = 0;

        /* calculate speed states */
        const double v = acadoVariables.x[i * NX + IDX_X_V];
        const double gamma = acadoVariables.x[i * NX + IDX_X_GAMMA];
        const double xi = acadoVariables.x[i * NX + IDX_X_XI];
        const double w_n = acadoVariables.od[i * NOD + IDX_OD_W];
        const double w_e = acadoVariables.od[i * NOD + IDX_OD_W+1];
        const double w_d = acadoVariables.od[i * NOD + IDX_OD_W+2];
        double speed_states[12];
        calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);

        /* sum detections in sliding window */
        for (int j = i; j < (i + len_sliding_window_); ++j)
        {
            if (occ_detect_slw_[j]>0) {
                add_unit_radial_distance_and_gradient(jac_r_unit, &r_unit_min, &f_min, &occ_count,
                                                      occ_slw_ + (j * NOCC + IDX_OCC_POS), occ_slw_ + (j * NOCC + IDX_OCC_NORMAL),
                                                      acadoVariables.x + (i * NX), speed_states, terrain_params_);
            }
        }

        /* calculate objective/jacobian */ // XXX: should probably have this encapsulated somehow
        double prio_r_fwd = 1.0;
        double sig_r_fwd;
        if (occ_count>0) {
            if (f_min) {
                // linear
                sig_r_fwd = 1.0 - log_sqrt_w_over_sig1_r_ * r_unit_min;
                jac_sig_r_fwd[0] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[0] / occ_count;
                jac_sig_r_fwd[1] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[1] / occ_count;
                jac_sig_r_fwd[2] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[2] / occ_count;
                jac_sig_r_fwd[3] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[3] / occ_count;
                jac_sig_r_fwd[4] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[4] / occ_count;
                jac_sig_r_fwd[5] = -log_sqrt_w_over_sig1_r_ * jac_r_unit[5] / occ_count;
            }
            else {
                // exponential
                sig_r_fwd = exp(-r_unit_min * log_sqrt_w_over_sig1_r_);
                jac_sig_r_fwd[0] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[0] / occ_count;
                jac_sig_r_fwd[1] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[1] / occ_count;
                jac_sig_r_fwd[2] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[2] / occ_count;
                jac_sig_r_fwd[3] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[3] / occ_count;
                jac_sig_r_fwd[4] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[4] / occ_count;
                jac_sig_r_fwd[5] = -log_sqrt_w_over_sig1_r_ * sig_r_fwd * jac_r_unit[5] / occ_count;
            }
            jac_sig_r_fwd[3] = 0.0; /* discourage mpc from using airspeed to combat costs */
            prio_r_fwd = constrain_double(r_unit_min, 0.0, 1.0);
        }
        prio_r_(i, IDX_PRIO_R_FWD) = prio_r_fwd;

        /* log cost and jacobian */
        od_(IDX_OD_SOFT_R, i) = sig_r_fwd;
        od_(IDX_OD_JAC_SOFT_R, i) = jac_sig_r_fwd[0];
        od_(IDX_OD_JAC_SOFT_R+1, i) = jac_sig_r_fwd[1];
        od_(IDX_OD_JAC_SOFT_R+2, i) = jac_sig_r_fwd[2];
        od_(IDX_OD_JAC_SOFT_R+3, i) = jac_sig_r_fwd[3];
        od_(IDX_OD_JAC_SOFT_R+4, i) = jac_sig_r_fwd[4];
        od_(IDX_OD_JAC_SOFT_R+5, i) = jac_sig_r_fwd[5];
    }
} // sumOcclusionDetections

void FwNMPC::evaluateExternalObjectives(const double *terrain_data)
{
    /* evaluate external objectives */
    double v_ray[3];
    for (int i = 0; i < ACADO_N +1; ++i)
    {
        /* soft aoa constraint */
        double sig_aoa, prio_aoa;
        double jac_sig_aoa[2];
        calculate_aoa_objective(&sig_aoa, jac_sig_aoa, &prio_aoa, acadoVariables.x + (i * ACADO_NX), aoa_params_);

        od_(IDX_OD_SOFT_AOA, i) = sig_aoa;
        od_(IDX_OD_JAC_SOFT_AOA, i) = jac_sig_aoa[0];
        od_(IDX_OD_JAC_SOFT_AOA+1, i) = jac_sig_aoa[1];

        prio_aoa_(i) = prio_aoa;

        /* soft height constraint */
        double sig_h, prio_h, h_terr;
        double jac_sig_h[4];
        calculate_height_objective(&sig_h, jac_sig_h, &prio_h, &h_terr, acadoVariables.x + (i * ACADO_NX), terrain_params_,
                                   terr_local_origin_n_, terr_local_origin_e_, map_height_, map_width_, map_resolution_, terrain_data);

        od_(IDX_OD_SOFT_H, i) = sig_h;
        od_(IDX_OD_JAC_SOFT_H, i) = jac_sig_h[0];
        od_(IDX_OD_JAC_SOFT_H+1, i) = jac_sig_h[1];
        od_(IDX_OD_JAC_SOFT_H+2, i) = jac_sig_h[2];
        od_(IDX_OD_JAC_SOFT_H+3, i) = jac_sig_h[3];

        prio_h_(i) = prio_h;

        /* calculate speed states */
        const double v = acadoVariables.x[i * ACADO_NX + IDX_X_V];
        const double gamma = acadoVariables.x[i * ACADO_NX + IDX_X_GAMMA];
        const double xi = acadoVariables.x[i * ACADO_NX + IDX_X_XI];
        const double w_n = acadoVariables.od[i * NOD + IDX_OD_W];
        const double w_e = acadoVariables.od[i * NOD + IDX_OD_W+1];
        const double w_d = acadoVariables.od[i * NOD + IDX_OD_W+2];
        double speed_states[12];
        calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);

        /* soft radial constraint */

        double dummy_pos[3];
        double dummy_normal[3];

        /* left ray */
        double sig_r_left, r_occ_left, prio_r_left;
        int occ_detected_left;
        double jac_sig_r_left[6];
        v_ray[0] = -speed_states[9];
        v_ray[1] = speed_states[10];
        v_ray[2] = 0.0;
        calculate_radial_objective(&sig_r_left, jac_sig_r_left, &r_occ_left, dummy_pos, dummy_normal, &prio_r_left, &occ_detected_left,
                                   v_ray, acadoVariables.x + (i * ACADO_NX), speed_states, terrain_params_, terr_local_origin_n_, terr_local_origin_e_,
                                   map_height_, map_width_, map_resolution_, terrain_data);

        /* right ray */
        double sig_r_right, r_occ_right, prio_r_right;
        int occ_detected_right;
        double jac_sig_r_right[6];
        v_ray[0] = speed_states[9];
        v_ray[1] = -speed_states[10];
        v_ray[2] = 0.0;
        calculate_radial_objective(&sig_r_right, jac_sig_r_right, &r_occ_right, dummy_pos, dummy_normal, &prio_r_right, &occ_detected_right,
                                   v_ray, acadoVariables.x + (i * ACADO_NX), speed_states, terrain_params_, terr_local_origin_n_, terr_local_origin_e_,
                                   map_height_, map_width_, map_resolution_, terrain_data);

        /* sum left, right, and forward detections */
        double jac_sig_r[6];
        jac_sig_r[0] = jac_sig_r_left[0] + jac_sig_r_right[0];
        jac_sig_r[1] = jac_sig_r_left[1] + jac_sig_r_right[1];
        jac_sig_r[2] = jac_sig_r_left[2] + jac_sig_r_right[2];
        jac_sig_r[3] = jac_sig_r_left[3] + jac_sig_r_right[3];
        jac_sig_r[4] = jac_sig_r_left[4] + jac_sig_r_right[4];
        jac_sig_r[5] = jac_sig_r_left[5] + jac_sig_r_right[5];

        /* add to objective cost / jacobian */
        od_(IDX_OD_SOFT_R, i) += sig_r_left + sig_r_right;
        od_(IDX_OD_JAC_SOFT_R, i) += jac_sig_r[0];
        od_(IDX_OD_JAC_SOFT_R+1, i) += jac_sig_r[1];
        od_(IDX_OD_JAC_SOFT_R+2, i) += jac_sig_r[2];
        od_(IDX_OD_JAC_SOFT_R+3, i) += jac_sig_r[3];
        od_(IDX_OD_JAC_SOFT_R+4, i) += jac_sig_r[4];
        od_(IDX_OD_JAC_SOFT_R+5, i) += jac_sig_r[5];

        /* prioritization */
        int occ_detected_fwd = occ_detect_slw_[len_sliding_window_-1+i];
        double prio_r = 1.0;
        if (!(one_over_sqrt_w_r_<0.0) && (occ_detected_fwd + occ_detected_left + occ_detected_right>0)) {

            /* take minimum unit radial distance */
            prio_r = (prio_r_(i, IDX_PRIO_R_FWD) < prio_r_left) ? prio_r_(i, IDX_PRIO_R_FWD) : prio_r_left;
            prio_r = (prio_r < prio_r_right) ? prio_r : prio_r_right;
        }
        prio_r_(i, IDX_PRIO_R_MAX) = prio_r;
        prio_r_(i, IDX_PRIO_R_LEFT) = prio_r_left;
        prio_r_(i, IDX_PRIO_R_RIGHT) = prio_r_right;

        /* velocity reference */
        double v_ref[3];
        calculate_velocity_reference(v_ref, &path_error_lat_, &path_error_lon_, acadoVariables.x + (i * NX), path_reference_, guidance_params_,
                                     speed_states, jac_sig_r, prio_r, path_type_);

        if (i < N) {
            acadoVariables.y[i * NY + IDX_Y_VN] = v_ref[0];
            acadoVariables.y[i * NY + IDX_Y_VE] = v_ref[1];
            acadoVariables.y[i * NY + IDX_Y_VD] = v_ref[2];
        }
        else {
            acadoVariables.yN[IDX_Y_VN] = v_ref[0];
            acadoVariables.yN[IDX_Y_VE] = v_ref[1];
            acadoVariables.yN[IDX_Y_VD] = v_ref[2];
        }
    }
} // evaluateExternalObjectives

void FwNMPC::prioritizeObjectives()
{
    // prioritizes select objectives by adapting specific weights (external to the model jacobian)

    // objective weighting through horizon (row-major array)
    for (int i=0; i<N; i++) {
        for (int j=IDX_Y_VN; j<IDX_Y_VD; j++) {
            acadoVariables.W[NY*NY*i+NY*j+j] *= prio_h_(i) * prio_r_(i,IDX_PRIO_R_MAX); // de-prioritize path following objectives when near terrain
        }
    }

    // end term objective weighting through horizon (row-major array)
    for (int j=IDX_Y_VN; j<IDX_Y_VD; j++) {
        acadoVariables.WN[NYN*j+j] *= prio_h_(N) * prio_r_(N,IDX_PRIO_R_MAX); // de-prioritize path following objectives when near terrain
    }
} // prioritizeObjectives

/* TERRAIN CHECKS  / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

void FwNMPC::lookup_terrain_idx(const double pos_n, const double pos_e, const double pos_n_origin,
                        const double pos_e_origin, const int map_height, const int map_width,
                        const double map_resolution, int *idx_q, double *dn, double *de)
{

    const int map_height_1 = map_height - 1;
    const int map_width_1 = map_width - 1;

    /* relative position / indices */
    const double rel_n = pos_n - pos_n_origin;
    const double rel_n_bar = rel_n / map_resolution;
    int idx_n = (int)(floor(rel_n_bar));
    const double rel_e = pos_e - pos_e_origin;
    const double rel_e_bar = rel_e / map_resolution;
    int idx_e = (int)(floor(rel_e_bar));

    /* interpolation weights */
    *dn = rel_n_bar-idx_n;
    *de = rel_e_bar-idx_e;

    /* cap ends */
    if (idx_n < 0) {
        idx_n = 0;
    }
    else if (idx_n > map_height_1) {
        idx_n = map_height_1;
    }
    if (idx_e < 0) {
        idx_e = 0;
    }
    else if (idx_e > map_width_1) {
        idx_e = map_width_1;
    }

    /* neighbors (north) */
    int q_n[4];
    if (idx_n >= map_height_1) {
        q_n[0] = map_height_1;
        q_n[1] = map_height_1;
        q_n[2] = map_height_1;
        q_n[3] = map_height_1;
    }
    else {
        q_n[0] = idx_n;
        q_n[1] = idx_n + 1;
        q_n[2] = idx_n;
        q_n[3] = idx_n + 1;
    }
    /* neighbors (east) */
    int q_e[4];
    if (idx_e >= map_width_1) {
        q_e[0] = map_width_1;
        q_e[1] = map_width_1;
        q_e[2] = map_width_1;
        q_e[3] = map_width_1;
    }
    else {
        q_e[0] = idx_e;
        q_e[1] = idx_e;
        q_e[2] = idx_e + 1;
        q_e[3] = idx_e + 1;
    }

    /* neighbors row-major indices */
    idx_q[0] = q_n[0]*map_width + q_e[0];
    idx_q[1] = q_n[1]*map_width + q_e[1];
    idx_q[2] = q_n[2]*map_width + q_e[2];
    idx_q[3] = q_n[3]*map_width + q_e[3];
}

/* check ray-triangle intersection */
int FwNMPC::intersect_triangle(double *d_occ, double *p_occ, double *n_occ,
                       const double r0[3], const double v_ray[3],
                       const double p1[3], const double p2[3], const double p3[3], const int v_dir) {
    /* following: "Fast, Minimum Storage Ray/Triangle Intersection",
     * Moeller et. al., Journal of Graphics Tools, Vol.2(1), 1997
     */

    /* NOTE: all vectors in here are E,N,U */

    /* find vectors for two edges sharing p1 */
    const double e1[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
    const double e2[3] = {p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]};

    /* begin calculating determinant - also used to calculate U parameter */
    double pvec[3];
    cross(pvec, v_ray, e2);

    /* we don't test for backwards culling here as, assuming this ray casting
     * algorithm is properly detecting the first occluding triangles, we should
     * not run into a case of a true "backwards" facing triangle (also the
     * current BR and TL definitions have opposite vertex spin definitions..
     * should probably change that in the future.. could maybe avoid a few
     * divisions of the determinant) */

    /* if the determinant is near zero, ray lies in the triangle plane */
    const double det = dot(e1, pvec);
    if (det > -EPSILON && det < EPSILON) {
        return 0;
    }

    /* divide the determinant (XXX: could possibly find a way to avoid this until the last minute possible..) */
    const double inv_det = 1.0 / det;

    /* calculate distance from p1 to ray origin */
    const double tvec[3] = {r0[0] - p1[0], r0[1] - p1[1], r0[2] - p1[2]};

    /* calculate u parameter and test bounds */
    const double u = dot(tvec, pvec) * inv_det;
    if (u < 0.0 || u > 1.0) {
        return 0;
    }

    /* prepare to test v parameter */
    double qvec[3];
    cross(qvec, tvec, e1);

    /* calculate v parameter and test bounds */
    const double v = dot(v_ray, qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0) {
        return 0;
    }

    /* calculate d_occ, scale parameters, ray intersects triangle */
    *d_occ = dot(e2, qvec) * inv_det;

    /* calculate and return intersection point */
    const double one_u_v = (1 - u - v);
    p_occ[0] = one_u_v * p1[0] + u * p2[0] + v * p3[0];
    p_occ[1] = one_u_v * p1[1] + u * p2[1] + v * p3[1];
    p_occ[2] = one_u_v * p1[2] + u * p2[2] + v * p3[2];

    /* calculate and return plane normal */
    cross(n_occ, e2, e1);
    const double one_over_norm_n_occ = 1.0/sqrt(dot(n_occ,n_occ));
    n_occ[0] *= v_dir * one_over_norm_n_occ;
    n_occ[1] *= v_dir * one_over_norm_n_occ;
    n_occ[2] *= v_dir * one_over_norm_n_occ;

    return 1;
}

/* cast ray through terrain map and determine the intersection point on any occluding trianglular surface */
int FwNMPC::castray(double *r_occ, double *p_occ, double *n_occ, double *p1, double *p2, double *p3,
            const double r0[3], const double r1[3], const double v[3],
            const double pos_n_origin, const double pos_e_origin, const int map_height,
            const int map_width, const double map_resolution, const double *terr_map) {

    /* INPUTS:
     *
     * (double) r0[3]             	start position (e,n,u) [m]
     * (double) r1[3]             	end position (e,n,u) [m]
     * (double) v[3]              	ray unit vector (e,n,u)
     * (double) pos_n_origin        northing origin of terrain map
     * (double) pos_e_origin        easting origin of terrain map
     * (double) map_resolution          	terrain discretization
     * (double) terr_map          	terrain map
     *
     * OUTPUTS:
     *
     * (int)   occ_detected         0=no detection, 1=BR triangle detected, 2=TL triangle detected
     * (double) d_occ           	distance to the ray-triangle intersection [m]
     * (double) p_occ[3]        	coord. of the ray-triangle intersection [m]
     * (double) n_occ[3]            plane unit normal vector
     * (double) p1[3]           	coord. of triangle vertex 1 (e,n,u) [m]
     * (double) p2[3]             	coord. of triangle vertex 2 (e,n,u) [m]
     * (double)	p3[3]           	coord. of triangle vertex 3 (e,n,u) [m]
     */

    const int map_height_1 = map_height - 1;
    const int map_width_1 = map_width - 1;

    /* initialize */
    int occ_detected = 0;

    /* relative (unit) start position */
    const double x0 = (r0[0] - pos_e_origin)/map_resolution;
    const double y0 = (r0[1] - pos_n_origin)/map_resolution;

    /* initial height */
    const double h0 = r0[2];

    /* vector for triangle intersect inputs */
    const double r0_rel[3] = {x0*map_resolution,y0*map_resolution,h0}; /*XXX: this origin subtracting/adding is inefficient.. pick one and go with it for this function

 /* relative end position */
    const double x1 = (r1[0] - pos_e_origin)/map_resolution;
    const double y1 = (r1[1] - pos_n_origin)/map_resolution;

    /* end height */
    const double h1 = r1[2];

    /* line deltas */
    const double dx = fabs(x1 - x0);
    const double dy = fabs(y1 - y0);

    /* initial cell origin */
    int x = (int)(floor(x0));
    int y = (int)(floor(y0));

    /* unit change in line length per x/y (see definition below) */
    double dt_dx;
    double dt_dy;

    /* change in height per unit line length (t) */
    const double dh = h1 - h0;

    /* number of cells we pass through */
    int n = fabs(floor(x1)-x) + fabs(floor(y1)-y) + 1; /*XXX: what is the real difference between this and using dx / dy? */

    /* initialize stepping criteria */
    double t_next_horizontal, t_last_horizontal;
    double t_next_vertical, t_last_vertical;
    double x_inc, y_inc;

    if (dx < 0.00001) {
        x_inc = 0.0;
        dt_dx = INFINITY;
        t_next_horizontal = INFINITY;
        t_last_horizontal = INFINITY;
    }
    else if (x1 > x0) {
        x_inc = 1.0;
        dt_dx = 1.0 / dx;
        t_next_horizontal = (x + 1.0 - x0) * dt_dx; /* remember x is "floor(x0)" here */
        t_last_horizontal = (x0 - x) * dt_dx;
    }
    else {
        x_inc = -1.0;
        dt_dx = 1.0 / dx;
        t_next_horizontal = (x0 - x) * dt_dx; /* remember x is "floor(x0)" here */
        t_last_horizontal = (x + 1.0 - x0) * dt_dx;
    }

    if (dy < 0.00001) {
        y_inc = 0.0;
        dt_dy = INFINITY;
        t_next_vertical = INFINITY;
        t_last_vertical = INFINITY;
    }
    else if (y1 > y0) {
        y_inc = 1.0;
        dt_dy = 1.0 / dy;
        t_next_vertical = (y + 1.0 - y0) * dt_dy; /* remember y is "floor(y0)" here */
        t_last_vertical = (y0 - y) * dt_dy;
    }
    else {
        y_inc = -1.0;
        dt_dy = 1.0 / dy;
        t_next_vertical = (y0 - y) * dt_dy; /* remember y is "floor(y0)" here */
        t_last_vertical = (y + 1.0 - y0) * dt_dy;
    }

    /* find cell intersection in opposite direction to initialize cell entrance
     * condition */
    bool last_step_was_vert = (t_last_vertical < t_last_horizontal);

    /* initialize entrance height */
    double h_entr = h0;

    /* for loop init */
    int ret;
    double t, h_exit, h_check;
    bool take_vert_step, check1, check2, check3, check4;

    /* check that start position is not already under the terrain */

    /* bound corner coordinates */
    int x_check = constrain_int(x, 0, map_width_1);
    int y_check = constrain_int(y, 0, map_height_1);
    int x_check1 = constrain_int(x_check+1, 0, map_width_1);
    int y_check1 = constrain_int(y_check+1, 0, map_height_1);
    /* convert to row-major indices */
    int idx_corner1 = y_check*map_width + x_check;
    int idx_corner2 = y_check1*map_width + x_check;
    int idx_corner3 = y_check1*map_width + x_check1;
    int idx_corner4 = y_check*map_width + x_check1;

    const double x0_unit = x0 - x;
    const double y0_unit = y0 - y;
    if (y0_unit > x0_unit) {
        /* check bottom-right triangle */
        if (x0_unit*(terr_map[idx_corner4]-terr_map[idx_corner1]) + y0_unit*(terr_map[idx_corner3] - terr_map[idx_corner4]) > h0) {
            return occ_detected;
        }
    }
    else {
        /* check top-left triangle */
        if (x0_unit*(terr_map[idx_corner3]-terr_map[idx_corner2]) + y0_unit*(terr_map[idx_corner2] - terr_map[idx_corner1]) > h0) {
            return occ_detected;
        }
    }

    /* cast the ray */
    int i;
    for (i = 0; i < n; i=i+1) {

        /* check the next step we will take and compute the exit height */
        if (t_next_vertical < t_next_horizontal) {
            /* next step is vertical */
            take_vert_step = true;
            t = t_next_vertical; /* current step */
            t_next_vertical = t_next_vertical + dt_dy;
        }
        else {
            /* next step is horizontal */
            take_vert_step = false;
            t = t_next_horizontal; /* current step */
            t_next_horizontal = t_next_horizontal + dt_dx;
        }

        /* take minimum of entrance and exit height for check */
        /* TODO: should be a way to get rid of this if statement by looking at dh outside for loop... */
        h_exit = h0 + dh * t;
        if (dh > 0.0) {
            h_check = h_entr;
        }
        else {
            h_check = h_exit;
        }
        h_entr = h_exit;

        /* bound corner coordinates */
        x_check = constrain_int(x, 0, map_width_1);
        y_check = constrain_int(y, 0, map_height_1);
        x_check1 = constrain_int(x_check+1, 0, map_width_1);
        y_check1 = constrain_int(y_check+1, 0, map_height_1);
        /* convert to row-major indices */
        idx_corner1 = y_check*map_width + x_check;
        idx_corner2 = y_check1*map_width + x_check;
        idx_corner3 = y_check1*map_width + x_check1;
        idx_corner4 = y_check*map_width + x_check1;
        /* check the four corners */
        check1 = terr_map[idx_corner1] > h_check; /* corner 1 (bottom left) */
        check2 = terr_map[idx_corner2] > h_check; /* corner 2 (top left) */
        check3 = terr_map[idx_corner3] > h_check; /* corner 3 (top right) */
        check4 = terr_map[idx_corner4] > h_check; /* corner 4 (bottom right) */

        /* check cell triangles */
        if (last_step_was_vert) { /* / / / / / / / / / / / / / / / / / / */
            /* vertical entrance step */

            if (take_vert_step) {
                /* next step is vertical */

                if (y_inc > 0) { /*TODO: should be able to get rid of a few of these ifs by making the decision outside the for loop... */
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
                else {
                    /* TL, BR */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    /* check bottom-right triangle corners */
                    if ((check1 || check4 || check3) && occ_detected==0) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
            }
            else  {/* - - - - - - - - - - - - - - - - - - - - - - - - - -*/
                /* next step is horizontal */

                if (y_inc > 0 && x_inc > 0) {
                    /* BR */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
                else if (y_inc < 0 && x_inc < 0) {
                    /* TL */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
                else {
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
            }
        }
        else { /* last step was horizontal / / / / / / / / / / / / / / / */
            if (take_vert_step) {
                /* next step is vertical */

                if (x_inc > 0) {
                    /* TL */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    if ((y_inc < 0) && (occ_detected==0)) {
                        /* BR */

                        /* check bottom-right triangle corners */
                        if (check1 || check4 || check3) {

                            /* set 3 corners */
                            p1[0] = map_resolution*x;
                            p1[1] = map_resolution*y;
                            p1[2] = terr_map[idx_corner1];
                            p2[0] = map_resolution*(x+1);
                            p2[1] = map_resolution*y;
                            p2[2] = terr_map[idx_corner4];
                            p3[0] = map_resolution*(x+1);
                            p3[1] = map_resolution*(y+1);
                            p3[2] = terr_map[idx_corner3];

                            /* check for ray-triangle intersection */
                            ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                            occ_detected += ret; /* =1 if detection */
                        }
                    }
                }
                else {
                    /* BR */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    if ((y > 0) && (occ_detected==0)) {
                        /* TL */

                        /* check top-left triangle corners */
                        if (check1 || check2 || check3) {

                            /* set 3 corners */
                            p1[0] = map_resolution*x;
                            p1[1] = map_resolution*y;
                            p1[2] = terr_map[idx_corner1];
                            p2[0] = map_resolution*x;
                            p2[1] = map_resolution*(y+1);
                            p2[2] = terr_map[idx_corner2];
                            p3[0] = map_resolution*(x+1);
                            p3[1] = map_resolution*(y+1);
                            p3[2] = terr_map[idx_corner3];

                            /* check for ray-triangle intersection */
                            ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                            occ_detected += ret*2; /* =2 if detection */
                        }
                    }
                }
            }
            else { /* - - - - - - - - - - - - - - - - - - - - - - - - - -*/
                /* next step is horizontal */

                if (x_inc > 0) {
                    /* TL, BR */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    /* check bottom-right triangle corners */
                    if ((check1 || check4 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
                else {
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
            }
        } /* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / */

        /* return if occlusion detected */
        if (occ_detected > 0) {
            return occ_detected;
        }

        /* actually take the step */
        if (take_vert_step) { /* (t_next_vertical < t_next_horizontal) */
            /* take a vertical step */
            y = y + y_inc;
        }
        else {
            /* take a horizontal step */
            x = x + x_inc;
        }
        last_step_was_vert = take_vert_step;
    }
    return occ_detected;
}

/* GUIDANCE  / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

/* calculate ground velocity reference */
void FwNMPC::calculate_velocity_reference(double *v_ref, double *e_lat, double *e_lon,
                                  const double *states,
                                  const double *path_reference,
                                  const double *guidance_params,
                                  const double *speed_states,
                                  const double *jac_sig_r,
                                  const double prio_r,
                                  const int path_type)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];
    const double gamma = states[4];
    const double xi = states[5];

    /* path reference */
    const double b_n = path_reference[0];
    const double b_e = path_reference[1];
    const double b_d = path_reference[2];

    /* guidance */
    const double T_b_lat = guidance_params[0];
    const double T_b_lon = guidance_params[1];
    const double gamma_app_max = guidance_params[2];
    bool use_occ_as_guidance = (guidance_params[3] > 0.5);

    /* speed states */
    const double vG_n = speed_states[3];
    const double vG_e = speed_states[4];
    const double vG_d = speed_states[5];

    /* path following - - - - - - - - - - - - - - - - - - - - - - - - - -*/

    double vP_n_unit = 0.0;
    double vP_e_unit = 0.0;
    double vP_d_unit = 0.0;

    if (path_type == 0) {
        /* loiter at fixed altitude */

        const double Gamma_p = 0.0;

        /* loiter direction (hijack chi_p param) */
        const double loiter_dir = (path_reference[4] < 0.0) ? -1.0 : 1.0;
        const double radius = (fabs(path_reference[4]) < 0.1) ? 0.1 : fabs(path_reference[4]);

        /* vector from circle center to aircraft */
        const double br_n = r_n-b_n;
        const double br_e = r_e-b_e;
        const double br_d = r_d-b_d;

        /* lateral-directional distance to circle center */
        const double dist_to_center = sqrt(br_n*br_n + br_e*br_e);

        /* norm of lateral-directional ground velocity */
        const double vG_lat = sqrt(vG_n*vG_n + vG_e*vG_e);

        double br_n_unit, br_e_unit;
        double p_n, p_e;
        if (dist_to_center < 0.1) {
            if (vG_lat < 0.1) {
                /* arbitrarily set the point in the northern top of the circle */
                br_n_unit = 1.0;
                br_e_unit = 0.0;

                /* closest point on circle */
                p_n = b_n + br_n_unit * radius;
                p_e = b_e + br_e_unit * radius;
            }
            else {
                /* set the point in the direction we are moving */
                br_n_unit = vG_n / vG_lat * 0.1;
                br_e_unit = vG_e / vG_lat * 0.1;

                /* closest point on circle */
                p_n = b_n + br_n_unit * radius;
                p_e = b_e + br_e_unit * radius;
            }
        }
        else {
            /* set the point in the direction of the aircraft */
            br_n_unit = br_n / dist_to_center;
            br_e_unit = br_e / dist_to_center;

            /* closest point on circle */
            p_n = b_n + br_n_unit * radius;
            p_e = b_e + br_e_unit * radius;
        }

        /* path tangent unit vector */
        const double tP_n_bar = -br_e_unit * loiter_dir;
        const double tP_e_bar = br_n_unit * loiter_dir;
        const double chi_p = atan2(tP_e_bar, tP_n_bar);

        /* position error */
        *e_lat = (r_n-p_n)*tP_e_bar - (r_e-p_e)*tP_n_bar;
        *e_lon = b_d - r_d;

        /* lateral-directional error boundary */
        const double e_b_lat = T_b_lat * sqrt(vG_n*vG_n + vG_e*vG_e);

        /* course approach angle */
        const double chi_app = atan(M_PI_2*(*e_lat)/e_b_lat);

        /* longitudinal error boundary */
        double e_b_lon;
        if (fabs(vG_d) < 1.0) {
            e_b_lon = T_b_lon * 0.5 * (1.0 + vG_d*vG_d); /* vG_d may be zero */
        }
        else {
            e_b_lon = T_b_lon * fabs(vG_d);
        }

        /* flight path approach angle */
        const double Gamma_app = -gamma_app_max * atan(M_PI_2*(*e_lon)/e_b_lon);

        /* normalized ground velocity setpoint */
        const double cos_gamma = cos(Gamma_p + Gamma_app);
        vP_n_unit = cos_gamma*cos(chi_p + chi_app);
        vP_e_unit = cos_gamma*sin(chi_p + chi_app);
        vP_d_unit = -sin(Gamma_p + Gamma_app);
    }
    else if (path_type == 1) {
        /* line */

        /* path direction */
        const double Gamma_p = path_reference[3];
        const double chi_p = path_reference[4];

        /* path tangent unit vector  */
        const double tP_n_bar = cos(chi_p);
        const double tP_e_bar = sin(chi_p);

        /* "closest" point on track */
        const double tp_dot_br = tP_n_bar*(r_n-b_n) + tP_e_bar*(r_e-b_e);
        const double tp_dot_br_n = tp_dot_br*tP_n_bar;
        const double tp_dot_br_e = tp_dot_br*tP_e_bar;
        const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
        const double p_d = b_d - p_lat*tan(Gamma_p);

        /* position error */
        *e_lat = (r_n-b_n)*tP_e_bar - (r_e-b_e)*tP_n_bar;
        *e_lon = p_d - r_d;

        /* lateral-directional error boundary */
        const double e_b_lat = T_b_lat * sqrt(vG_n*vG_n + vG_e*vG_e);

        /* course approach angle */
        const double chi_app = atan(M_PI_2*(*e_lat)/e_b_lat);

        /* longitudinal error boundary */
        double e_b_lon;
        if (fabs(vG_d) < 1.0) {
            e_b_lon = T_b_lon * 0.5 * (1.0 + vG_d*vG_d); /* vG_d may be zero */
        }
        else {
            e_b_lon = T_b_lon * fabs(vG_d);
        }

        /* flight path approach angle */
        const double Gamma_app = -gamma_app_max * atan(M_PI_2*(*e_lon)/e_b_lon);

        /* normalized ground velocity setpoint */
        const double cos_gamma = cos(Gamma_p + Gamma_app);
        vP_n_unit = cos_gamma*cos(chi_p + chi_app);
        vP_e_unit = cos_gamma*sin(chi_p + chi_app);
        vP_d_unit = -sin(Gamma_p + Gamma_app);
    }
    else {
        /* unknown */
        /* fly north.. */

        /* position error */
        *e_lat = 0.0;
        *e_lon = 0.0;

        /* normalized ground velocity setpoint */
        vP_n_unit = 1.0;
        vP_e_unit = 0.0;
        vP_d_unit = 0.0;
    }

    if (use_occ_as_guidance) {
        /* terrain avoidance velocity setpoint */
        const double norm_jac_sig_r = sqrt(jac_sig_r[0]*jac_sig_r[0] + jac_sig_r[1]*jac_sig_r[1] + jac_sig_r[2]*jac_sig_r[2]);
        const double one_over_norm_jac_sig_r = (norm_jac_sig_r > 0.0001) ? 1.0/norm_jac_sig_r : 10000.0;
        const double v_occ_n_unit = -jac_sig_r[0] * one_over_norm_jac_sig_r;
        const double v_occ_e_unit = -jac_sig_r[1] * one_over_norm_jac_sig_r;
        const double v_occ_d_unit = -jac_sig_r[2] * one_over_norm_jac_sig_r;

        /* velocity errors */
        v_ref[0] = vP_n_unit * prio_r + (1.0-prio_r) * v_occ_n_unit;
        v_ref[1] = vP_e_unit * prio_r + (1.0-prio_r) * v_occ_e_unit;
        v_ref[2] = vP_d_unit * prio_r + (1.0-prio_r) * v_occ_d_unit;
    }
    else {
        /* velocity errors */
        v_ref[0] = vP_n_unit;
        v_ref[1] = vP_e_unit;
        v_ref[2] = vP_d_unit;
    }
}

/* SOFT CONSTRAINTS / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

/* soft angle of attack jacobian (exponential) */
void jacobian_sig_aoa_exp(double *jac,
        const double delta_aoa, const double log_sqrt_w_over_sig1_aoa,
        const double sig_aoa_m, const double sig_aoa_p) {

    /* w.r.t.:
     * gamma
     * theta
     */

    const double t2 = 1.0/delta_aoa;
    const double t3 = log_sqrt_w_over_sig1_aoa*sig_aoa_m*t2;
    jac[0] = t3-log_sqrt_w_over_sig1_aoa*sig_aoa_p*t2;
    jac[1] = -t3+log_sqrt_w_over_sig1_aoa*sig_aoa_p*t2;
}

/* height terrain jacobian (linear) */
void FwNMPC::jacobian_sig_h_lin(double *jac,
        const double de, const double delta_h, const double delta_y,
        const double  h1, const double h12, const double h2,
        const double h3, const double h34, const double h4,
        const double log_sqrt_w_over_sig1_h, const double sgn_e,
        const double sgn_n, const double map_resolution, const double xi)
{

    /* w.r.t.:
     * r_n
     * r_e
     * r_d
     * xi
     */

    const double t2 = 1.0/map_resolution;
    const double t3 = 1.0/delta_h;
    const double t4 = de-1.0;
    const double t5 = cos(xi);
    const double t6 = sin(xi);
    const double t7 = delta_y*sgn_n*t2*t5;
    const double t8 = delta_y*sgn_e*t2*t6;
    const double t9 = log_sqrt_w_over_sig1_h*t3;
    jac[0] = -t9*(de*(h3*t2-h4*t2)-t4*(h1*t2-h2*t2));
    jac[1] = -t9*(h12*t2-h34*t2);
    jac[2] = t9;
    jac[3] = -t9*(t7*(de*(h3-h4)-t4*(h1-h2))+t8*(-h12+h34));
}

/* height terrain jacobian (exponential) */
void FwNMPC::jacobian_sig_h_exp(double *jac,
        const double de, const double delta_h, const double delta_y,
        const double h1, const double h12, const double h2, const double h3,
        const double h34, const double h4, const double log_sqrt_w_over_sig1_h,
        const double sgn_e, const double sgn_n, const double sig_h,
        const double map_resolution, const double xi)
{

    /* w.r.t.:
     * r_n
     * r_e
     * r_d
     * xi
     */

    const double t2 = 1.0/map_resolution;
    const double t3 = 1.0/delta_h;
    const double t4 = de-1.0;
    const double t5 = cos(xi);
    const double t6 = sin(xi);
    const double t7 = log_sqrt_w_over_sig1_h*sig_h*t3;
    const double t8 = sgn_e*t6;
    const double t9 = de*sgn_n*t5;
    const double t10 = sgn_n*t4*t5;
    jac[0] = -t7*(de*(h3*t2-h4*t2)-t4*(h1*t2-h2*t2));
    jac[1] = -t7*(h12*t2-h34*t2);
    jac[2] = t7;
    jac[3] = delta_y*t2*t7*(h12*t8 - h34*t8 - h3*t9 + h4*t9 + h1*t10 - h2*t10);
}

/* jacobian of unit radial distance */
void FwNMPC::jacobian_r_unit(double *jac,
        const double delta_r, const double gamma, const double k_delta_r, const double k_r_offset,
        const double n_occ_e, const double n_occ_h, const double n_occ_n,
        const double r_unit, const double v,
        const double v_ray_e, const double v_ray_h, const double v_ray_n,
        const double v_rel, const double xi)
{

    /* w.r.t.:
    * r_n
    * r_e
    * r_d
    * v
    * gamma
    * xi
    */

    const double t2 = 1.0/delta_r;
    const double t3 = n_occ_e*v_ray_e;
    const double t4 = n_occ_h*v_ray_h;
    const double t5 = n_occ_n*v_ray_n;
    const double t6 = t3+t4+t5;
    const double t7 = 1.0/t6;
    const double t8 = cos(gamma);
    const double t9 = k_delta_r*r_unit;
    const double t10 = k_r_offset+t9;
    const double t11 = sin(gamma);
    const double t12 = cos(xi);
    const double t13 = sin(xi);
    jac[0] = -n_occ_n*t2*t7;
    jac[1] = -n_occ_e*t2*t7;
    jac[2] = n_occ_h*t2*t7;
    jac[3] = t2*t10*v_rel*(t11*v_ray_h+t8*t13*v_ray_e+t8*t12*v_ray_n)*-2.0;
    jac[4] = t2*t10*v*v_rel*(-t8*v_ray_h+t11*t13*v_ray_e+t11*t12*v_ray_n)*2.0;
    jac[5] = t2*t8*t10*v*v_rel*(t12*v_ray_e-t13*v_ray_n)*-2.0;
}

/* calculate soft angle of attack objective */
void FwNMPC::calculate_aoa_objective(double *sig_aoa, double *jac_sig_aoa, double *prio_aoa,
                             const double *states, const double *aoa_params)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /* states */
    /*const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];*/
    const double gamma = states[4];
    /*const double xi = states[5];
    const double phi = states[6];*/
    const double theta = states[7];
    /*const double n_p = states[8];*/

    /* angle of attack soft constraint */
    const double delta_aoa = aoa_params[0];
    const double aoa_m = aoa_params[1];
    const double aoa_p = aoa_params[2];
    const double log_sqrt_w_over_sig1_aoa = aoa_params[3];
    const double one_over_sqrt_w_aoa = aoa_params[4];

    /* angle of attack */
    const double aoa = theta - gamma;
    *sig_aoa = 0.0;
    *prio_aoa = 1.0;
    jac_sig_aoa[0] = 0.0;
    jac_sig_aoa[1] = 0.0;

    /* angle of attack objective / jacobian - - - - - - - - - - - - - - -*/

    if (!(one_over_sqrt_w_aoa<0.0)) {

        /* upper bound */
        const double sig_aoa_p = (aoa - aoa_p < 0.0)
                                 ? exp((aoa - aoa_p)/delta_aoa*log_sqrt_w_over_sig1_aoa)
                                 : 1.0 + log_sqrt_w_over_sig1_aoa/delta_aoa * (aoa - aoa_p);

        /* lower bound */
        const double sig_aoa_m = (aoa - aoa_m > 0.0)
                                 ? exp(-(aoa - aoa_m)/delta_aoa*log_sqrt_w_over_sig1_aoa)
                                 : 1.0 - log_sqrt_w_over_sig1_aoa/delta_aoa * (aoa - aoa_m);

        /* combined */
        *sig_aoa = sig_aoa_p + sig_aoa_m;

        /* jacobian */
        if (aoa - aoa_p > 0.0) {
            /* upper linear jacobian */
            jac_sig_aoa[0] = -log_sqrt_w_over_sig1_aoa/delta_aoa; /* gamma */
            jac_sig_aoa[1] = log_sqrt_w_over_sig1_aoa/delta_aoa; /* theta */
        }
        else if (aoa - aoa_m < 0.0) {
            /* lower linear jacobian */
            jac_sig_aoa[0] = log_sqrt_w_over_sig1_aoa/delta_aoa; /* gamma */
            jac_sig_aoa[1] = -log_sqrt_w_over_sig1_aoa/delta_aoa; /* theta */
        }
        else {
            /* exponential jacobian */
            jacobian_sig_aoa_exp(jac_sig_aoa, delta_aoa, log_sqrt_w_over_sig1_aoa,
                                 sig_aoa_m, sig_aoa_p);
        }

        /* prioritization */
        *prio_aoa = 1.0; /* TODO: consider putting this back */
    }
}

/* calculate soft height objective */
void FwNMPC::calculate_height_objective(double *sig_h, double *jac_sig_h, double *prio_h, double *h_terr,
                                const double *states, const double *terr_params, const double terr_local_origin_n,
                                const double terr_local_origin_e, const int map_height, const int map_width,
                                const double map_resolution, const double *terr_map)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    /*const double v = states[3];
    const double gamma = states[4];*/
    const double xi = states[5];
    /*const double phi = states[6];
    const double theta = states[7];
    const double n_p = states[8];*/

    /* height params */
    const double h_offset = terr_params[0];
    const double delta_h = terr_params[1];
    const double delta_y = 0.0;
    const double log_sqrt_w_over_sig1_h = terr_params[2];
    const double one_over_sqrt_w_h = terr_params[3];

    /* INTERMEDIATE CALCULATIONS - - - - - - - - - - - - - - - - - - - - */

    const double sin_xi = sin(xi);
    const double cos_xi = cos(xi);

    /* CALCULATE OBJECTIVE - - - - - - - - - - - - - - - - - - - - - - - */

    /* init */
    const double h = -r_d;
    *sig_h = 0.0;
    double sig_h_temp = 0.0;
    *prio_h = 1.0;
    jac_sig_h[0] = 0.0;
    jac_sig_h[1] = 0.0;
    jac_sig_h[2] = 0.0;
    jac_sig_h[3] = 0.0;
    double jac_sig_h_temp[4] = {0.0, 0.0, 0.0, 0.0};

    /* if not disabled by weight */
    if (!(one_over_sqrt_w_h<0.0)) {

        /* lookup 2.5d grid (CENTER) - - - - - - - - - - - - - - - - - - */
        int idx_q[4];
        double dn, de;
        double sgn_n = 0.0;
        double sgn_e = 0.0;
        lookup_terrain_idx(r_n, r_e, terr_local_origin_n, terr_local_origin_e, map_height, map_width, map_resolution, idx_q, &dn, &de);

        /* bi-linear interpolation */
        double h12 = (1-dn)*terr_map[idx_q[0]] + dn*terr_map[idx_q[1]];
        double h34 = (1-dn)*terr_map[idx_q[2]] + dn*terr_map[idx_q[3]];
        double h_terr_temp = (1-de)*h12 + de*h34;
        *h_terr = h_terr_temp;

        /* objective / jacobian */
        const double sig_input = (h - h_terr_temp - h_offset)/delta_h;
        if (sig_input < 0.0) {
            /* linear */
            *sig_h = 1.0 + -log_sqrt_w_over_sig1_h * sig_input;

            jacobian_sig_h_lin(jac_sig_h,
                               de, delta_h, delta_y,
                               terr_map[idx_q[0]], h12, terr_map[idx_q[1]],
                               terr_map[idx_q[2]], h34, terr_map[idx_q[3]],
                               log_sqrt_w_over_sig1_h, sgn_e,
                               sgn_n, map_resolution, xi);
        }
        else {
            /* exponential */
            *sig_h = exp(-sig_input*log_sqrt_w_over_sig1_h);

            jacobian_sig_h_exp(jac_sig_h,
                               de, delta_h, delta_y,
                               terr_map[idx_q[0]], h12, terr_map[idx_q[1]],
                               terr_map[idx_q[2]], h34, terr_map[idx_q[3]],
                               log_sqrt_w_over_sig1_h, sgn_e, sgn_n, *sig_h,
                               map_resolution, xi);
        }

        /* prioritization */
        *prio_h = constrain_double(sig_input, 0.0, 1.0);
    }
}

/* calculate soft radial objective */
void FwNMPC::calculate_radial_objective(double *sig_r, double *jac_sig_r, double *r_occ,
                                double *p_occ, double *n_occ, double *prio_r, int *occ_detected,
                                const double *v_ray, const double *states, const double *speed_states,
                                const double *terr_params, const double terr_local_origin_n, const double terr_local_origin_e,
                                const int map_height, const int map_width, const double map_resolution, const double *terr_map)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];
    const double gamma = states[4];
    const double xi = states[5];
    /*const double phi = states[6];
    const double theta = states[7];
    const double n_p = states[8];*/

    /* speed states */
    const double vG_n = speed_states[3];
    const double vG_e = speed_states[4];
    const double vG_d = speed_states[5];
    const double vG_sq = speed_states[6];
    const double vG_norm = speed_states[7];
    const double vG_n_unit = speed_states[9];
    const double vG_e_unit = speed_states[10];
    const double vG_d_unit = speed_states[11];

    /* radial params */
    const double r_offset = terr_params[4];
    const double delta_r0 = terr_params[5];
    const double k_r_offset = terr_params[6];
    const double k_delta_r = terr_params[7];
    const double log_sqrt_w_over_sig1_r = terr_params[8];
    const double one_over_sqrt_w_r = terr_params[9];


    /* CALCULATE OBJECTIVE - - - - - - - - - - - - - - - - - - - - - - - */

    /* init */
    *sig_r = 0.0;
    *prio_r = 1.0;
    jac_sig_r[0] = 0.0;
    jac_sig_r[1] = 0.0;
    jac_sig_r[2] = 0.0;
    jac_sig_r[3] = 0.0;
    jac_sig_r[4] = 0.0;
    jac_sig_r[5] = 0.0;

    /* cast ray along ground speed vector to check for occlusions */

    /* init */
    double p1[3];
    double p2[3];
    double p3[3];

    /* relative velocity */
    const double vG_vec[3] = {vG_e, vG_n, -vG_d};
    double v_rel = dot(v_ray, vG_vec); /* in ENU */
    if (v_rel < 0.0) v_rel = 0.0;
    const double v_rel_sq = v_rel*v_rel;

    /* radial buffer zone */
    const double delta_r = delta_r0 + v_rel_sq * k_delta_r;

    /* adjusted radial offset */
    const double r_offset_1 = r_offset + v_rel_sq * k_r_offset;

    /* ray length */
    const double d_ray = delta_r + r_offset_1 + map_resolution;

    /* ray start ENU */
    const double r0[3] = {r_e, r_n, -r_d};
    /* ray end ENU */
    const double r1[3] = {r0[0] + v_ray[0] * d_ray, r0[1] + v_ray[1] * d_ray, r0[2] + v_ray[2] * d_ray};

    /* cast the ray */
    *occ_detected = castray(r_occ, p_occ, n_occ, p1, p2, p3, r0, r1, v_ray,
                            terr_local_origin_n, terr_local_origin_e, map_height, map_width,
                            map_resolution, terr_map);

    /* shift occlusion origin */
    p_occ[0] = p_occ[0] + terr_local_origin_e;
    p_occ[1] = p_occ[1] + terr_local_origin_n;

    if (!(one_over_sqrt_w_r<0.0) && (*occ_detected>0)) {

        const double r_unit = (*r_occ - r_offset_1)/delta_r;

        double jac_r_unit[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

        jacobian_r_unit(jac_r_unit,
                        delta_r, gamma, k_delta_r, k_r_offset,
                        n_occ[0], n_occ[2], n_occ[1],
                        r_unit, v,
                        v_ray[0], v_ray[2], v_ray[1],
                        v_rel, xi);

        /* objective / jacobian */
        if (r_unit < 0.0) {
            /* linear */
            *sig_r = 1.0 - log_sqrt_w_over_sig1_r * r_unit;
            jac_sig_r[0] = -log_sqrt_w_over_sig1_r * jac_r_unit[0];
            jac_sig_r[1] = -log_sqrt_w_over_sig1_r * jac_r_unit[1];
            jac_sig_r[2] = -log_sqrt_w_over_sig1_r * jac_r_unit[2];
            jac_sig_r[3] = -log_sqrt_w_over_sig1_r * jac_r_unit[3];
            jac_sig_r[4] = -log_sqrt_w_over_sig1_r * jac_r_unit[4];
            jac_sig_r[5] = -log_sqrt_w_over_sig1_r * jac_r_unit[5];
        }
        else {
            /* exponential */
            *sig_r = exp(-r_unit*log_sqrt_w_over_sig1_r);
            const double mult_ = -log_sqrt_w_over_sig1_r * r_unit;
            jac_sig_r[0] = mult_ * jac_r_unit[0];
            jac_sig_r[1] = mult_ * jac_r_unit[1];
            jac_sig_r[2] = mult_ * jac_r_unit[2];
            jac_sig_r[3] = mult_ * jac_r_unit[3];
            jac_sig_r[4] = mult_ * jac_r_unit[4];
            jac_sig_r[5] = mult_ * jac_r_unit[5];
        }
        jac_sig_r[3] = 0.0; /* discourage mpc from using airspeed to combat costs */

        /* prioritization */
        *prio_r = constrain_double(r_unit, 0.0, 1.0);
    }
}

/* calculate unit radial distance and gradient */
void FwNMPC::add_unit_radial_distance_and_gradient(double *jac_r_unit, double *r_unit_min, bool *f_min, int *occ_count,
        double *p_occ, double *n_occ, const double *states, const double *speed_states, const double *terr_params)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];
    const double gamma = states[4];
    const double xi = states[5];

    /* speed states */
    const double vG_n = speed_states[3];
    const double vG_e = speed_states[4];
    const double vG_d = speed_states[5];

    /* radial params */
    const double r_offset = terr_params[4];
    const double delta_r0 = terr_params[5];
    const double k_r_offset = terr_params[6];
    const double k_delta_r = terr_params[7];
    const double log_sqrt_w_over_sig1_r = terr_params[8];
    const double one_over_sqrt_w_r = terr_params[9];

    /* CALCULATE OBJECTIVE - - - - - - - - - - - - - - - - - - - - - - - */

    double r_occ_vec[3] = {r_e - p_occ[0], r_n - p_occ[1], -r_d - p_occ[2]};

    /* check if we are in front of obstacle */
    if (dot(r_occ_vec, n_occ) > 0) {
        /* calculate the unit distance and gradient */

        /* update detection count */
        *occ_count += 1;

        /* distance to obstacle */
        const double r_occ = sqrt(dot(r_occ_vec,r_occ_vec));

        /* normalize ray vector (NOTE: flip (-) to point TOWARDS obsctacle) */
        r_occ_vec[0] = -r_occ_vec[0] / r_occ;
        r_occ_vec[1] = -r_occ_vec[1] / r_occ;
        r_occ_vec[2] = -r_occ_vec[2] / r_occ;

        /* relative velocity */
        const double vG_vec[3] = {vG_e, vG_n, -vG_d};
        double v_rel = dot(r_occ_vec, vG_vec); /* in ENU */
        if (v_rel < 0.0) v_rel = 0.0;
        const double v_rel_sq = v_rel*v_rel;

        /* radial buffer zone */
        const double delta_r = delta_r0 + v_rel_sq * k_delta_r;

        /* adjusted radial offset */
        const double r_offset_1 = r_offset + v_rel_sq * k_r_offset;

        /* calculate unit distance */
        const double r_unit = (r_occ - r_offset_1)/delta_r;

        /* calculate gradient */
        double jac_r_unit_temp[6];
        jacobian_r_unit(jac_r_unit_temp,
                        delta_r, gamma, k_delta_r, k_r_offset,
                        n_occ[0], n_occ[2], n_occ[1],
                        r_unit, v,
                        r_occ_vec[0], r_occ_vec[2], r_occ_vec[1],
                        v_rel, xi);

        /* update minimum unit distance */
        if ((r_unit < *r_unit_min) || *occ_count == 1) {
            *r_unit_min = r_unit;
            *f_min = (r_unit < 0.0);
        }

        /* add */
        jac_r_unit[0] += jac_r_unit_temp[0];
        jac_r_unit[1] += jac_r_unit_temp[1];
        jac_r_unit[2] += jac_r_unit_temp[2];
        jac_r_unit[3] += jac_r_unit_temp[3];
        jac_r_unit[4] += jac_r_unit_temp[4];
        jac_r_unit[5] += jac_r_unit_temp[5];
    }
}

/* cast a ray along the ground speed vector and return any detection point and normal */
void FwNMPC::get_occ_along_gsp_vec(double *p_occ, double *n_occ, double *r_occ, int *occ_detected,
                           const double *states, const double *speed_states, const double *terr_params,
                           const double terr_local_origin_n, const double terr_local_origin_e,
                           const int map_height, const int map_width, const double map_resolution, const double *terr_map)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */

    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];
    const double gamma = states[4];
    const double xi = states[5];

    /* speed states */
    const double vG_sq = speed_states[6];
    const double vG_n_unit = speed_states[9];
    const double vG_e_unit = speed_states[10];
    const double vG_d_unit = speed_states[11];

    /* radial params */
    const double r_offset = terr_params[4];
    const double delta_r0 = terr_params[5];
    const double k_r_offset = terr_params[6];
    const double k_delta_r = terr_params[7];
    const double log_sqrt_w_over_sig1_r = terr_params[8];
    const double one_over_sqrt_w_r = terr_params[9];

    /* cast ray along ground speed vector to check for occlusions */

    /* init */
    double p1[3];
    double p2[3];
    double p3[3];

    /* ray vector */
    const double v_ray[3] = {vG_e_unit, vG_n_unit, -vG_d_unit};

    /* radial buffer zone */
    const double delta_r = delta_r0 + vG_sq * k_delta_r;

    /* adjusted radial offset */
    const double r_offset_1 = r_offset + vG_sq * k_r_offset;

    /* ray length */
    const double d_ray = delta_r + r_offset_1 + map_resolution;

    /* ray start ENU */
    const double r0[3] = {r_e, r_n, -r_d};
    /* ray end ENU */
    const double r1[3] = {r0[0] + v_ray[0] * d_ray, r0[1] + v_ray[1] * d_ray, r0[2] + v_ray[2] * d_ray};

    /* cast the ray */
    *occ_detected = castray(r_occ, p_occ, n_occ, p1, p2, p3, r0, r1, v_ray,
                            terr_local_origin_n, terr_local_origin_e, map_height, map_width,
                            map_resolution, terr_map);

    /* shift occlusion origin */
    p_occ[0] = p_occ[0] + terr_local_origin_e;
    p_occ[1] = p_occ[1] + terr_local_origin_n;
}

void FwNMPC::filterControlReference()
{
    double tau_u;
    nmpc_.getParam("/nmpc/filt/tau_u", tau_u); // control reference time constant

    // first order filter on reference horizon driven by currently applied control held through horizon
    u_ref_ = (u_ * Eigen::Matrix<double, 1, N>::Ones() - u_ref_) / tau_u / getLoopRate() + u_ref_;

} // filterControlReference

void FwNMPC::filterTerrainCostJacobian()
{
    double tau_terr;
    nmpc_.getParam("/nmpc/filt/tau_terr", tau_terr); // terrain jacobian time constant

    // first order filter on terrain based jacobian
    for (int i=0; i<N+1; i++) {
        for (int j=IDX_OD_OBJ; j<NOD; j++) {
            acadoVariables.od[NOD * i + j] = (od_(j, i) - acadoVariables.od[NOD * i + j]) / tau_terr / getLoopRate() + acadoVariables.od[NOD * i + j];
        }
    }

} // filterControlReference

/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/
/* NMPC FUNCTIONS  / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

void FwNMPC::applyControl()
{
    // apply first NU controls
    double u_T = acadoVariables.u[IDX_U_U_T];
    double phi_ref = acadoVariables.u[IDX_U_PHI_REF];
    double theta_ref = acadoVariables.u[IDX_U_THETA_REF];
    if (isnan(u_T) || isnan(phi_ref) || isnan(theta_ref)) {
        // probably solver issue
        u_T = 0.0;
        phi_ref = 0.0;
        theta_ref = 0.0;
        re_init_horizon_ = true;
        obctrl_status_ = 11; // 11 (arbitrarily) for NaNs -- TODO: enumerate
    }
    else {
        // constrain to bounds XXX: adapt these if necessary once we start playing with the constraints
        u_T = constrain_double(u_T, lb_(IDX_U_U_T), ub_(IDX_U_U_T));
        phi_ref = constrain_double(phi_ref, lb_(IDX_U_PHI_REF), ub_(IDX_U_PHI_REF));
        theta_ref = constrain_double(theta_ref, lb_(IDX_U_THETA_REF), ub_(IDX_U_THETA_REF));
    }

    u_(IDX_U_U_T) = u_T;
    u_(IDX_U_PHI_REF) = phi_ref;
    u_(IDX_U_THETA_REF) = theta_ref;
}

int FwNMPC::nmpcIteration()
{
    // < -- START objective pre-evaluation timer
    ros::Time t_iter_start = ros::Time::now();

    obctrl_status_ = 0;
    ros::Duration t_elapsed;

    // initialize returns
    int RET[2] = {0, 0};

    // check offboard status
    if (!offboard_mode_) re_init_horizon_ = true;

    /* updates */

    // < -- START objective pre-evaluation timer
    ros::Time t_preeval_start = ros::Time::now();

    updateAcadoX0(); // new measurements
    updateAcadoW(); // new weights (not including prioritization)
    updateAcadoConstraints(); // new constraints
    if (!re_init_horizon_)  filterControlReference(); // from previously applied control

    if (re_init_horizon_) {
        initHorizon();
        re_init_horizon_ = false;
    }

    /* objective pre-evaluation */

    // pre-evaluate select objectives (external to ACADO model)
    preEvaluateObjectives();

    // objective prioritization
    prioritizeObjectives();

    // update objective references
    updateAcadoY();

    // update online data (*DEPENDS on preEvaluateObjectives)
    filterTerrainCostJacobian(); // time delayed terrain cost jacobians
    updateAcadoOD();

    t_elapsed = ros::Time::now() - t_preeval_start;
    uint64_t t_preeval = t_elapsed.toNSec()/1000;
    // END objective pre-evaluation timer -- >

    /* ACADO solver functions */

    // SOLVER: prepare ACADO workspace for next iteration step
    // < -- START preparation step timer
    ros::Time t_prep_start = ros::Time::now();
    RET[0] = acado_preparationStep();
    if ( RET[0] != 0 ) {
        ROS_ERROR("nmpc iteration: preparation step error, code %d", RET[0]);
        obctrl_status_ = RET[0];
        re_init_horizon_ = true;
        first_yaw_received_ = false;
    }
    t_elapsed = ros::Time::now() - t_prep_start;
    uint64_t t_prep = t_elapsed.toNSec()/1000;
    // END preparation step timer -- >

    // SOLVER: perform the feedback step
    // < -- START feedback step timer
    ros::Time t_fb_start = ros::Time::now();
    RET[1] = acado_feedbackStep();
    if ( RET[1] != 0 ) {
        ROS_ERROR("nmpc iteration: feedback step error, code %d", RET[1]);
        obctrl_status_ = RET[1]; //TODO: find a way that doesnt overwrite the other one...
        re_init_horizon_ = true;
    }
    t_elapsed = ros::Time::now() - t_fb_start;
    uint64_t t_fb = t_elapsed.toNSec()/1000;
    // END feedback step timer -- >

    /* publishing */

    // send controls to vehicle
    applyControl();
    if (!re_init_horizon_) publishControls(u_(IDX_U_U_T), u_(IDX_U_PHI_REF), u_(IDX_U_THETA_REF)); // don't publish if a re_init is necessary
    t_elapsed = ros::Time::now() - t_last_ctrl_;
    uint64_t t_ctrl = t_elapsed.toNSec()/1000;
    t_last_ctrl_ = ros::Time::now(); // record directly after publishing control

    // store current nmpc states
    publishNMPCStates();

    // record timing / nmpc info
    publishNMPCInfo(t_iter_start, t_ctrl, t_preeval, t_prep, t_fb);

    return 0;
} // nmpcIteration

void FwNMPC::updateAcadoConstraints()
{
    // update hard constraints

    // lower bounds
    nmpc_.getParam("/nmpc/lb/u_T", lb_(IDX_U_U_T));
    nmpc_.getParam("/nmpc/lb/phi_ref", lb_(IDX_U_PHI_REF));
    lb_(IDX_U_PHI_REF) *= DEG_TO_RAD;
    nmpc_.getParam("/nmpc/lb/theta_ref", lb_(IDX_U_THETA_REF));
    lb_(IDX_U_THETA_REF) *= DEG_TO_RAD;

    // upper bounds
    nmpc_.getParam("/nmpc/ub/u_T", ub_(IDX_U_U_T));
    nmpc_.getParam("/nmpc/ub/phi_ref", ub_(IDX_U_PHI_REF));
    ub_(IDX_U_PHI_REF) *= DEG_TO_RAD;
    nmpc_.getParam("/nmpc/ub/theta_ref", ub_(IDX_U_THETA_REF));
    ub_(IDX_U_THETA_REF) *= DEG_TO_RAD;

    // TODO: adapt constraints as necessary, e.g. for landing

    for (int i=0; i<N; i++) {
        acadoVariables.lbValues[NU * i + IDX_U_U_T] = lb_(IDX_U_U_T);
        acadoVariables.lbValues[NU * i + IDX_U_PHI_REF] = lb_(IDX_U_PHI_REF);
        acadoVariables.lbValues[NU * i + IDX_U_THETA_REF] = lb_(IDX_U_THETA_REF);

        acadoVariables.ubValues[NU * i + IDX_U_U_T] = ub_(IDX_U_U_T);
        acadoVariables.ubValues[NU * i + IDX_U_PHI_REF] = ub_(IDX_U_PHI_REF);
        acadoVariables.ubValues[NU * i + IDX_U_THETA_REF] = ub_(IDX_U_THETA_REF);
    }

} // updateAcadoConstraints

void FwNMPC::updateAcadoOD()
{
    // air density
    od_.block(IDX_OD_RHO, 0, 1, N+1).setConstant(calcAirDensity());

    // wind
    od_.block(IDX_OD_W, 0, 3, N+1) = x0_wind_ * Eigen::Matrix<double, 1, N+1>::Ones();

    // control-augmented model dynamics
    double temp_param;
    nmpc_.getParam("/nmpc/od/tau_phi", temp_param);
    od_.block(IDX_OD_TAU_PHI, 0, 1, N+1).setConstant(temp_param);
    nmpc_.getParam("/nmpc/od/tau_theta", temp_param);
    od_.block(IDX_OD_TAU_THETA, 0, 1, N+1).setConstant(temp_param);
    nmpc_.getParam("/nmpc/od/k_phi", temp_param);
    od_.block(IDX_OD_K_PHI, 0, 1, N+1).setConstant(temp_param);
    nmpc_.getParam("/nmpc/od/k_theta", temp_param);
    od_.block(IDX_OD_K_THETA, 0, 1, N+1).setConstant(temp_param);

    // prop delay
    nmpc_.getParam("/nmpc/od/tau_n", temp_param);
    od_.block(IDX_OD_TAU_N, 0, 1, N+1).setConstant(temp_param);

    // symmetric flaps setting
    od_.block(IDX_OD_DELTA_F, 0, 1, N+1).setConstant(flapsToRad(flaps_normalized_));

    // NOTE: pre-evaluated objectives/jacobians set in updateAcadoODObjectives -- this should be run BEFORE this update function

    Eigen::Map<Eigen::Matrix<double, NOD, N+1>>(const_cast<double*>(acadoVariables.od)) = od_;
} // updateAcadoOD

void FwNMPC::updateAcadoW()
{
    // NOTE: this does not yet include any weight priorities

    // state squared output scales (inverse of -- save a few divisions..)
    nmpc_.getParam("/nmpc/inv_y_scale_sq/vn", inv_y_scale_sq_(IDX_Y_VN));
    nmpc_.getParam("/nmpc/inv_y_scale_sq/vd", inv_y_scale_sq_(IDX_Y_VD));
    nmpc_.getParam("/nmpc/inv_y_scale_sq/ve", inv_y_scale_sq_(IDX_Y_VE));
    nmpc_.getParam("/nmpc/inv_y_scale_sq/v", inv_y_scale_sq_(IDX_Y_V));
    nmpc_.getParam("/nmpc/inv_y_scale_sq/phi", inv_y_scale_sq_(IDX_Y_PHI));
    nmpc_.getParam("/nmpc/inv_y_scale_sq/theta", inv_y_scale_sq_(IDX_Y_THETA));
    nmpc_.getParam("/nmpc/inv_y_scale_sq/soft_aoa", inv_y_scale_sq_(IDX_Y_SOFT_AOA));
    nmpc_.getParam("/nmpc/inv_y_scale_sq/soft_h", inv_y_scale_sq_(IDX_Y_SOFT_H));
    nmpc_.getParam("/nmpc/inv_y_scale_sq/soft_r", inv_y_scale_sq_(IDX_Y_SOFT_R));
    // control squared output scales
    nmpc_.getParam("/nmpc/inv_y_scale_sq/u_T", inv_y_scale_sq_(IDX_Y_U_T));
    nmpc_.getParam("/nmpc/inv_y_scale_sq/phi_ref", inv_y_scale_sq_(IDX_Y_PHI_REF));
    nmpc_.getParam("/nmpc/inv_y_scale_sq/theta_ref", inv_y_scale_sq_(IDX_Y_THETA_REF));

    // state weights
    nmpc_.getParam("/nmpc/w/vn", w_(IDX_Y_VN));
    nmpc_.getParam("/nmpc/w/ve", w_(IDX_Y_VE));
    nmpc_.getParam("/nmpc/w/vd", w_(IDX_Y_VD));
    nmpc_.getParam("/nmpc/w/v", w_(IDX_Y_V));
    nmpc_.getParam("/nmpc/w/phi", w_(IDX_Y_PHI));
    nmpc_.getParam("/nmpc/w/theta", w_(IDX_Y_THETA));
    nmpc_.getParam("/nmpc/w/soft_aoa", w_(IDX_Y_SOFT_AOA));
    nmpc_.getParam("/nmpc/w/soft_h", w_(IDX_Y_SOFT_H));
    nmpc_.getParam("/nmpc/w/soft_r", w_(IDX_Y_SOFT_R));
    // control weights
    nmpc_.getParam("/nmpc/w/u_T", w_(IDX_Y_U_T));
    nmpc_.getParam("/nmpc/w/phi_ref", w_(IDX_Y_PHI_REF));
    nmpc_.getParam("/nmpc/w/theta_ref", w_(IDX_Y_THETA_REF));

    // objective weighting through horizon (row-major array)
    for (int i=0; i<N; i++) {
        for (int j=0; j<NY; j++) {
            acadoVariables.W[NY*NY*i+NY*j+j] = w_(j) * inv_y_scale_sq_(j);
        }
    }

    // end term objective weighting through horizon (row-major array)
    for (int j=0; j<NY; j++) {
        acadoVariables.WN[NYN*j+j] = w_(j) * inv_y_scale_sq_(j);
    }

} // updateAcadoW

void FwNMPC::updateAcadoX0()
{
    /* ASSUMPTIONS:
     * - yaw is assumed = to heading (i.e. no sideslip)
     * - we need to unwrap heading because it is integrated through the horizon
     */

    // position
    x0_.segment(IDX_X_POS,3) = x0_pos_;

    // velocity axis
    Eigen::Vector3d airsp_vec = x0_vel_ - x0_wind_;
    const double airsp = airsp_vec.norm();
    double airsp_thres;
    nmpc_.getParam("/nmpc/veh/airsp_thres", airsp_thres);
    if (airsp < airsp_thres) {
        // airspeed is too low - bound it to minimum value
        x0_(IDX_X_V) = airsp_thres;
        if (airsp < 0.1) { // XXX: magic number
            // if reeeally zero - define airspeed vector in yaw direction with zero flight path angle
            x0_(IDX_X_GAMMA) = 0.0; // flight path angle
            x0_(IDX_X_XI) = unwrapHeading(x0_euler_(2)); // heading
        }
        else {
            x0_(IDX_X_GAMMA) = -asin(airsp_vec(2)/airsp); // flight path angle
            x0_(IDX_X_XI) = unwrapHeading(atan2(airsp_vec(1),airsp_vec(0))); // heading
        }
    }
    else {
        x0_(IDX_X_V) = airsp; // airspeed
        x0_(IDX_X_GAMMA) = -asin(airsp_vec(2)/airsp); // flight path angle
        x0_(IDX_X_XI) = unwrapHeading(atan2(airsp_vec(1),airsp_vec(0))); // heading
    }

    // attitude
    x0_(IDX_X_PHI) = x0_euler_(0); // roll
    x0_(IDX_X_THETA) = x0_euler_(1); // pitch

    // prop speed
    if (re_init_horizon_) {
        // take current state
        const double throt = mapPX4ToThrot(px4_throt_, airsp, x0_(IDX_X_THETA) - x0_(IDX_X_GAMMA));
        n_prop_virt_ = mapThrotToPropSpeed(throt, airsp, x0_(IDX_X_THETA) - x0_(IDX_X_GAMMA));
    }
    else {
        // propagate from last state
        propagateVirtPropSpeed(u_(IDX_U_U_T), acadoVariables.x[IDX_X_V], acadoVariables.x[IDX_X_THETA] - acadoVariables.x[IDX_X_GAMMA]);
    }
    x0_(IDX_X_NPROP) = n_prop_virt_;

    Eigen::Map<Eigen::Matrix<double, NX, 1>>(const_cast<double*>(acadoVariables.x0)) = x0_;
} // updateAcadoX0

void FwNMPC::updateAcadoY()
{
    // NOTE: run AFTER pre-evaluation of objectives

    // airspeed reference
    double airsp_ref;
    nmpc_.getParam("/nmpc/y_ref/v",airsp_ref);
    y_.block(IDX_Y_V, 0, 1, N).setConstant(airsp_ref);
    yN_(IDX_Y_V) = airsp_ref;

    // attitude reference
    y_.block(IDX_Y_PHI, 0, 1, N) = u_ref_.block(IDX_U_PHI_REF, 0, 1, N);
    y_.block(IDX_Y_THETA, 0, 1, N) = u_ref_.block(IDX_U_THETA_REF, 0, 1, N);
    yN_(IDX_Y_PHI) = u_ref_(IDX_U_PHI_REF, N);
    yN_(IDX_Y_THETA) = u_ref_(IDX_U_THETA_REF, N);

    // control objective references
    y_.block(IDX_Y_U_T, 0, 1, N) = u_ref_.block(IDX_U_U_T, 0, 1, N);
    y_.block(IDX_Y_PHI_REF, 0, 1, N) = u_ref_.block(IDX_U_PHI_REF, 0, 1, N);
    y_.block(IDX_Y_THETA_REF, 0, 1, N) = u_ref_.block(IDX_U_THETA_REF, 0, 1, N);

    // NOTE: unit ground velocity references set in objective pre-evaluation

    Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) = y_;
    Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) = yN_;
} // updateAcadoY

/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/
/* INITIALIZATION  / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

void FwNMPC::initACADOVars()
{
    x0_pos_.setZero();
    x0_vel_ << 15.0, 0.0, 0.0;
    x0_euler_.setZero();
    x0_wind_.setZero();

    for (int i = 0; i < MAX_SIZE_TERR_ARRAY; ++i) terr_array_[i] = 0.0;

    x0_.setZero();
    u_.setZero();
    u_ref_.setZero();
    y_.setZero();
    yN_.setZero();
    inv_y_scale_sq_.setZero();
    w_.setZero();
    od_.setZero();

    for (int i = 0; i < N+LEN_SLIDING_WINDOW_MAX; ++i) occ_detect_slw_[i] = 0;
    for (int i = 0; i < ((N+LEN_SLIDING_WINDOW_MAX) * NOCC); ++i) occ_slw_[i] = 0.0;

    updateAcadoX0();
    updateAcadoConstraints();
    updateAcadoW();
    updateAcadoOD();
    updateAcadoY();
} // initACADOVars

void FwNMPC::initHorizon()
{
    if (re_init_horizon_) {
        // we are re-initializing the horizon possibly in-air, propagate current state measurements forward

        for (int i = 0; i < N; ++i) {

            // propagate position linearly at current ground velocity
            for (int j = IDX_X_POS; j < IDX_X_POS+3; ++j) {
                acadoVariables.x[NX * i + j] = x0_pos_(j) + x0_vel_(j) * getTimeStep() * i;
            }

            // hold velocity axis constant through horizon
            acadoVariables.x[NX * i + IDX_X_V] = x0_(IDX_X_V);
            acadoVariables.x[NX * i + IDX_X_GAMMA] = x0_(IDX_X_GAMMA);
            acadoVariables.x[NX * i + IDX_X_XI] = x0_(IDX_X_XI);

            // hold attitude constant (within bounds) through horizon
            double roll = constrain_double(x0_(IDX_X_PHI), lb_(IDX_U_PHI_REF), ub_(IDX_U_PHI_REF));
            double pitch = constrain_double(x0_(IDX_X_THETA), lb_(IDX_U_THETA_REF), ub_(IDX_U_THETA_REF));
            acadoVariables.x[NX * i + IDX_X_PHI] = roll;
            acadoVariables.x[NX * i + IDX_X_THETA] = pitch;

            // hold current throttle constant through horizon
            double throt = mapPX4ToThrot(px4_throt_, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
            double n_prop = mapThrotToPropSpeed(throt, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
            acadoVariables.x[NX * i + IDX_X_NPROP] = n_prop;

            // controls
            acadoVariables.u[NU * i + IDX_U_U_T] = throt;
            acadoVariables.u[NU * i + IDX_U_PHI_REF] = 0.0;
            acadoVariables.u[NU * i + IDX_U_THETA_REF] = 0.0;
        }

        // propagate position linearly at current ground velocity
        for (int j = IDX_X_POS; j < IDX_X_POS+3; ++j) {
            acadoVariables.x[NX * N + j] = x0_pos_(j) + x0_vel_(j) * getTimeStep() * N;
        }

        // hold velocity axis constant through horizon
        acadoVariables.x[NX * N + IDX_X_V] = x0_(IDX_X_V);
        acadoVariables.x[NX * N + IDX_X_GAMMA] = x0_(IDX_X_GAMMA);
        acadoVariables.x[NX * N + IDX_X_XI] = x0_(IDX_X_XI);

        // hold attitude constant (within bounds) through horizon
        double roll = constrain_double(x0_(IDX_X_PHI), lb_(IDX_U_PHI_REF), ub_(IDX_U_PHI_REF));
        double pitch = constrain_double(x0_(IDX_X_THETA), lb_(IDX_U_THETA_REF), ub_(IDX_U_THETA_REF));
        acadoVariables.x[NX * N + IDX_X_PHI] = roll;
        acadoVariables.x[NX * N + IDX_X_THETA] = pitch;

        // hold current throttle constant through horizon
        double throt = mapPX4ToThrot(px4_throt_, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
        double n_prop = mapThrotToPropSpeed(throt, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
        acadoVariables.x[NX * N + IDX_X_NPROP] = n_prop;

        ROS_ERROR("re-init horizon: propagate states with zero attitude reference and constant throttle through horizon");
    }
    else {
        // this is the first initialization, the values don't really matter here...

        for (int i = 0; i < N; ++i) {

            // propagate position linearly at current ground velocity
            for (int j = IDX_X_POS; j < IDX_X_POS+3; ++j) {
                acadoVariables.x[NX * i + j] = x0_pos_(j) + x0_vel_(j) * getTimeStep() * i;
            }

            // hold velocity axis constant through horizon
            acadoVariables.x[NX * i + IDX_X_V] = x0_(IDX_X_V);
            acadoVariables.x[NX * i + IDX_X_GAMMA] = x0_(IDX_X_GAMMA);
            acadoVariables.x[NX * i + IDX_X_XI] = x0_(IDX_X_XI);

            // zero roll, assume zero angle of attack, and hold pitch constant
            acadoVariables.x[NX * i + IDX_X_PHI] = 0.0;
            acadoVariables.x[NX * i + IDX_X_THETA] = x0_(IDX_X_GAMMA);

            // prop off
            acadoVariables.x[NX * i + IDX_X_NPROP] = 0.0;

            // controls
            acadoVariables.u[NU * i + IDX_U_U_T] = 0.0;
            acadoVariables.u[NU * i + IDX_U_PHI_REF] = 0.0;
            acadoVariables.u[NU * i + IDX_U_THETA_REF] = 0.0;
        }

        // propagate position linearly at current ground velocity
        for (int j = IDX_X_POS; j < IDX_X_POS+3; ++j) {
            acadoVariables.x[NX * N + j] = x0_pos_(j) + x0_vel_(j) * getTimeStep() * N;
        }

        // hold velocity axis constant through horizon
        acadoVariables.x[NX * N + IDX_X_V] = x0_(IDX_X_V);
        acadoVariables.x[NX * N + IDX_X_GAMMA] = x0_(IDX_X_GAMMA);
        acadoVariables.x[NX * N + IDX_X_XI] = x0_(IDX_X_XI);

        // zero roll, assume zero angle of attack, and hold pitch constant
        acadoVariables.x[NX * N + IDX_X_PHI] = 0.0;
        acadoVariables.x[NX * N + IDX_X_THETA] = x0_(IDX_X_GAMMA);

        // prop off
        acadoVariables.x[NX * N + IDX_X_NPROP] = 0.0;

        ROS_ERROR("init horizon: hold states and controls constant through horizon");
    }

} // initHorizon

int FwNMPC::initNMPC()
{
    /* initialize ACADO variables */
    initACADOVars();
    ROS_ERROR("init nmpc: NMPC states initialized");

    /* initialize the solver */
    int RET = acado_initializeSolver();

    /* initialize the horizon */ //XXX: should this be before initializing the solver? will this anyway happen again in the first time through the iteration loop?
    initHorizon();

    return RET;
} // initNMPC

/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/
/* NODE  / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

void FwNMPC::shutdown() {
    ROS_INFO("Shutting down NMPC...");
    ros::shutdown();
} // shutdown

int main(int argc, char **argv)
{

    /* initialize node */
    ros::init(argc, argv, "fw_nmpc");
    fw_nmpc::FwNMPC nmpc;

    ros::spinOnce();

    /* wait for required subscriptions */
    nmpc.checkSubs();

    /* initialize states, params, and solver */
    int ret = nmpc.initNMPC();
    if (ret != 0) {
        ROS_ERROR("init nmpc: error in qpOASES QP solver.");
        return 1;
    }

    /*
     * nmpc loop
     */
    ros::Rate nmpc_rate(nmpc.getLoopRate());
    ROS_ERROR("fw nmpc: entering NMPC loop");
    while (ros::ok()) {

        /* empty callback queues */
        ros::spinOnce();

        /* nmpc iteration step */
        ret = nmpc.nmpcIteration();
        if (ret != 0) {
            ROS_ERROR("nmpc iteration: error in qpOASES QP solver.");
            return 1;
        }

        /* sleep */
        nmpc_rate.sleep();
    }

    ROS_ERROR("fw nmpc: closing...");
    return 0;
} // main
