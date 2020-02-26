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

#include <fw_nmpc/nonlinear_mpc.h>

#include <math.h>
#include <algorithm>

namespace fw_nmpc {

NonlinearMPC::NonlinearMPC() :
    control_config_nh_("~/control"),
    serverControl(control_config_nh_),
    guidance_config_nh_("~/guidance"),
    serverGuidance(guidance_config_nh_),
    manual_control_config_nh_("~/manual_control"),
    serverManualControl(manual_control_config_nh_),
    occlusion_detection_config_nh_("~/occlusion_detection"),
    serverOcclusionDetection(occlusion_detection_config_nh_),
    soft_constraints_config_nh_("~/soft_constraints"),
    serverSoftConstraints(soft_constraints_config_nh_),
    flaps_normalized_(0.0),
    home_lat_(0),
    home_lon_(0),
    home_alt_(0),
    landed_state_(0), // 0=undefined
    offboard_mode_(false),
    px4_mode_("MANUAL"),
    px4_throt_(0.0),
    static_pressure_pa_(101325.0),
    temperature_c_(25.0),
    x0_pos_(Eigen::Vector3d::Zero()),
    x0_vel_(Eigen::Vector3d::Zero()),
    x0_euler_(Eigen::Vector3d::Zero()),
    x0_wind_(Eigen::Vector3d::Zero()),
    n_prop_virt_(0.0),
    map_origin_north_(0.0),
    map_origin_east_(0.0),
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
    ray_casting_interval_(1),
    len_occ_data_(ACADO_N+1),
    len_occ_window_(1),
    len_occ_buffer_(1),
    rtd_node_interval_(1),
    occ_window_start_(Eigen::Matrix<int, ACADO_N+1, 1>::Zero()),
    nearest_ray_origin_(Eigen::Matrix<int, ACADO_N+1, 1>::Zero()),
    surfel_radius_(0.0),
    terrain_alt_horizon_(Eigen::Matrix<double, ACADO_N+1, 1>::Zero()),
    inv_prio_aoa_(Eigen::Matrix<double, ACADO_N+1, 1>::Ones()),
    inv_prio_hagl_(Eigen::Matrix<double, ACADO_N+1, 1>::Ones()),
    inv_prio_rtd_(Eigen::Matrix<double, ACADO_N+1, 1>::Ones()),
    t_last_ctrl_(0),
    home_pos_valid_(false),
    px4_connected_(false),
    obctrl_status_(OffboardControlStatus::NOMINAL),
    err_code_preparation_step_(0),
    err_code_feedback_step_(0),
    re_init_horizon_(false),
    grid_map_valid_(false),
    terrain_was_disabled_(true)
{
    ROS_INFO("Instance of NMPC created");

    /* subscribers */
    act_sub_ = nmpc_.subscribe("/mavros/target_actuator_control", 1, &NonlinearMPC::actCb, this);
    grid_map_sub_ = nmpc_.subscribe("map_mpc", 1, &NonlinearMPC::gridMapCb, this);
    home_pos_sub_ = nmpc_.subscribe("/mavros/home_position/home", 1, &NonlinearMPC::homePosCb, this);
    imu_sub_ = nmpc_.subscribe("/mavros/imu/data", 1, &NonlinearMPC::imuCb, this);
    local_pos_sub_ = nmpc_.subscribe("/mavros/global_position/local", 1, &NonlinearMPC::localPosCb, this);
    man_ctrl_sub_ = nmpc_.subscribe("/mavros/manual_control/control", 1, &NonlinearMPC::manCtrlCb, this);
    static_pres_sub_ = nmpc_.subscribe("/mavros/imu/static_pressure", 1, &NonlinearMPC::staticPresCb, this);
    sys_status_sub_ = nmpc_.subscribe("/mavros/state", 1, &NonlinearMPC::sysStatusCb, this);
    sys_status_ext_sub_ = nmpc_.subscribe("/mavros/extended_state", 1, &NonlinearMPC::sysStatusExtCb, this);
    temp_c_sub_ = nmpc_.subscribe("/mavros/imu/temperature_imu", 1, &NonlinearMPC::tempCCb, this);
    wind_est_sub_ = nmpc_.subscribe("/mavros/wind_estimation", 1, &NonlinearMPC::windEstCb, this);

    /* publishers */
    att_sp_pub_ = nmpc_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 1);
    nmpc_aux_out_pub_ = nmpc_.advertise<fw_ctrl::NMPCAuxOut>("/nmpc/aux_out",10);
    nmpc_controls_pub_ = nmpc_.advertise<fw_ctrl::NMPCControls>("/nmpc/controls",10);
    nmpc_guidance_path_pub_ = nmpc_.advertise<nav_msgs::Path>("/nmpc/guidance_path",1);
    nmpc_info_pub_ = nmpc_.advertise<fw_ctrl::NMPCInfo>("/nmpc/info",10);
    nmpc_meas_pub_ = nmpc_.advertise<fw_ctrl::NMPCMeasurements>("/nmpc/measurements",10);
    nmpc_obj_ref_pub_ = nmpc_.advertise<fw_ctrl::NMPCObjRef>("/nmpc/obj_ref",10);
    nmpc_objN_ref_pub_ = nmpc_.advertise<fw_ctrl::NMPCObjNRef>("/nmpc/objN_ref",10);
    nmpc_occ_normals_pub_ = nmpc_.advertise<visualization_msgs::Marker>("/nmpc/occ_normals",1);
    nmpc_occ_planes_pub_ = nmpc_.advertise<visualization_msgs::Marker>("/nmpc/occ_planes",1);
    nmpc_occ_rays_pub_ = nmpc_.advertise<visualization_msgs::Marker>("/nmpc/occ_rays",1);
    nmpc_online_data_pub_ = nmpc_.advertise<fw_ctrl::NMPCOnlineData>("/nmpc/online_data",10);
    nmpc_states_pub_ = nmpc_.advertise<fw_ctrl::NMPCStates>("/nmpc/states",10);
    nmpc_traj_pred_pub_ = nmpc_.advertise<nav_msgs::Path>("/nmpc/traj_pred",1);
    obctrl_status_pub_ = nmpc_.advertise<std_msgs::Int32>("/nmpc/status", 1);
    thrust_pub_ = nmpc_.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 1);
    unit_gnd_vel_ref_pub_ = nmpc_.advertise<visualization_msgs::MarkerArray>("/nmpc/vg_unit_ref",1);

    /* dynamic reconfigure */

    // The dynamic reconfigure server for control parameters
    serverControl.setCallback(boost::bind(&NonlinearMPC::parametersCallbackControl, this, _1, _2));

    // The dynamic reconfigure server for guidance parameters
    serverGuidance.setCallback(boost::bind(&NonlinearMPC::parametersCallbackGuidance, this, _1, _2));

    // The dynamic reconfigure server for manual control parameters
    serverManualControl.setCallback(boost::bind(&NonlinearMPC::parametersCallbackManualControl, this, _1, _2));

    // The dynamic reconfigure server for occlusion detection parameters
    serverOcclusionDetection.setCallback(boost::bind(&NonlinearMPC::parametersCallbackOcclusionDetection, this, _1, _2));

    // The dynamic reconfigure server for soft constraints parameters
    serverSoftConstraints.setCallback(boost::bind(&NonlinearMPC::parametersCallbackSoftConstraints, this, _1, _2));

    /*
     * <taken from ftf frame conversions in MAVROS>
     *
     * Static quaternions needed for rotating between ENU and NED frames
     * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
     * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
     * aircraft and base_link: +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
     * aircraft and base_link: Fto Forward, Left, Up (base_link) frames.
     */
    ned_enu_q_.setRPY(M_PI, 0.0, M_PI_2);
    aircraft_baselink_q_.setRPY(M_PI, 0.0, 0.0);

    // initialize manual velocity control setpoints (arbitrary)
    manual_control_sp_.airspeed = 14.0;
    manual_control_sp_.roll = 0.0;
    manual_control_sp_.bearing = 0.0;
    manual_control_sp_.bearing_rate = 0.0;
    manual_control_sp_.unit_delta_alt_rate = 0.0;
    manual_control_sp_.unit_vel(0) = 1.0;
    manual_control_sp_.unit_vel(0) = 0.0;
    manual_control_sp_.unit_vel(0) = 0.0;
    manual_control_sp_.alt = 100.0;
    manual_control_sp_.rel_alt = 100.0;

    // check that we have a large enough maximum on the occlusion detector data length
    if (OcclusionDetector::LEN_DATA_MAX < ACADO_N+1) {
        ROS_ERROR("NonlinearMPC: horizon exceeds maximum data length in occlusion buffer");
        shutdown();
    }
}

/*
    CALLBACKS
*/

void NonlinearMPC::actCb(const mavros_msgs::ActuatorControl::ConstPtr& msg) // actuator control target msg callback
{
    if (msg->group_mix == msg->PX4_MIX_FLIGHT_CONTROL) {
        px4_throt_ = (double)msg->controls[3];
        flaps_normalized_ = (double)msg->controls[4];
    }
} // actCb

void NonlinearMPC::gridMapCb(const grid_map_msgs::GridMap& msg) // grid map msg callback
{
    bool ret;
    grid_map_valid_ = true;

    // get incoming terrain data
    ret = grid_map::GridMapRosConverter::fromMessage(msg, terrain_map_);
    if (!ret) {
        ROS_ERROR("grid map cb: failed to convert msg");
        grid_map_valid_ = false;
    }

    // grid map center (assuming constant level-northing orientation in world frame)
    double map_center_north_world = msg.info.pose.position.y; // N ("world" frame) = y (grid map frame) [m]
    double map_center_east_world = msg.info.pose.position.x; // E ("world" frame) = x (grid map frame) [m]

    // get local terrain map origin (top-right corner of array)
    map_origin_north_ = map_center_north_world - msg.info.length_y / 2.0;
    map_origin_east_ = map_center_east_world - msg.info.length_x / 2.0;

    // map dimensions
    map_resolution_ = msg.info.resolution;
    map_height_ = int(std::round(msg.info.length_y / map_resolution_)); // north size //XXX: should be able to get these sizes directly from the floatmultiarray info..
    map_width_ = int(std::round(msg.info.length_x / map_resolution_)); // east size

    // convert local terrain map to row major array // TODO: should be a better way than needing to create these and pass as pointer every time.. e.g. pass the eigen matrix or gridmap itself
    terrain_map_["elevation"].reverseInPlace(); // TODO: sync the format of the acado functions with grid map.. or use grid map in the solver functions themselves
    int broadcast_world_map_tf;
    nmpc_.getParam("/nmpc/mapping_with_ground_truth", broadcast_world_map_tf);
    if (home_pos_valid_ && broadcast_world_map_tf) {
        // "world" -> "map" (where nmpc is operating) //XXX: THIS IS MESSY
        terrain_map_["elevation"].array() -= translate_world_to_home_(2);
        map_origin_north_ -= translate_world_to_home_(0);
        map_origin_east_ -= translate_world_to_home_(1);
    }
    if (map_height_*map_width_ > MAX_SIZE_TERR_ARRAY) {
        ROS_ERROR("grid map cb: received terrain map exceeds maximum allowed size");
        grid_map_valid_ = false;
    }
    else {
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>>(map_array_, map_height_, map_width_) = terrain_map_["elevation"].cast<double>();
    }
} // gridMapCb

void NonlinearMPC::homePosCb(const mavros_msgs::HomePosition::ConstPtr& msg) // home position msg callback
{
    home_lat_ = (double)msg->geo.latitude; // [deg]
    home_lon_ = (double)msg->geo.longitude; // [deg]
    home_alt_ = (double)msg->geo.altitude; // [m] AMSL

    /* check if we are feeding mapping pipeline with ground truth from simulation and need to broadcast the appropriate transform */
    int broadcast_world_map_tf;
    nmpc_.getParam("/nmpc/mapping_with_ground_truth", broadcast_world_map_tf);
    if (home_pos_valid_ && broadcast_world_map_tf) {
        // we've received the home position and can now broadcast a transform from gazebo "world" to "map" frame
        // only executed if feeding mapping pipeline with ground truth sim

        double lat_world_origin, lon_world_origin, alt_world_origin;
        nmpc_.getParam("/nmpc/sim/lat", lat_world_origin);
        nmpc_.getParam("/nmpc/sim/lon", lon_world_origin);
        nmpc_.getParam("/nmpc/sim/alt", alt_world_origin);

        double north, east;
        ll2NE(north, east, home_lat_, home_lon_, lat_world_origin, lon_world_origin); // small angle approx for home vs world offset

        static tf::TransformBroadcaster br; // static broadcaster
        tf::Transform transform; // redefine transform each time home position (possibly) changes

        transform.setOrigin( tf::Vector3(east, north, home_alt_ - alt_world_origin) );
        translate_world_to_home_(0) = north; // XXX: THIS IS MESSY
        translate_world_to_home_(1) = east; // XXX: THIS IS MESSY
        translate_world_to_home_(2) = home_alt_ - alt_world_origin; // XXX: THIS IS MESSY
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));

        static bool first_broadcast = true;
        if (first_broadcast) {
            ROS_ERROR("home position callback: broadcasting world to map tf with translation offset (%.3f, %.3f, %.3f)", east, north, home_alt_ - alt_world_origin);
            first_broadcast = false;
        }
    }
} // homePosCb

void NonlinearMPC::imuCb(const sensor_msgs::Imu::ConstPtr& msg) // imu msg callback
{
    tf::Quaternion q_enu;
    tf::quaternionMsgToTF(msg->orientation, q_enu); // MAVROS only publishes ENU orientation

    // convert to NED
    tf::Quaternion q_ned = ned_enu_q_ * q_enu * aircraft_baselink_q_; // XXX: maybe should just add a publisher for the ned quaternion already stored in MAVROS?

    double roll, pitch, yaw;
    tf::Matrix3x3(q_ned).getRPY(roll, pitch, yaw);

    x0_euler_(0) = roll;
    x0_euler_(1) = pitch;
    x0_euler_(2) = yaw;
} // imuCb

void NonlinearMPC::localPosCb(const nav_msgs::Odometry::ConstPtr& msg) // local position msg callback
{
    // NOTE: local positions referenced to home position origin in "map" parent frame

    // ENU -> NED
    x0_pos_(0) = msg->pose.pose.position.y;
    x0_pos_(1) = msg->pose.pose.position.x;
    x0_pos_(2) = -msg->pose.pose.position.z;

    // END (XXX: this is a MAVROS error .. probably to do with PX4 vs adrupilot sending different MAVLINK data) -> NED
    x0_vel_(0) = msg->twist.twist.linear.y;
    x0_vel_(1) = msg->twist.twist.linear.x;
    x0_vel_(2) = msg->twist.twist.linear.z;

} // localPosCb

void NonlinearMPC::manCtrlCb(const mavros_msgs::ManualControl::ConstPtr& msg) // manual control msg callback
{
    static bool was_disabled = true;
    static bool mode_changed = false;
    static ManualControlTypes last_mode = ManualControlTypes::DIRECTION;
    static ros::Duration t_elapsed(0);
    static ros::Time t_last(0);

    // manual unit velocity control //NOTE: this is currently ground relative velocity control -- should perhaps make an option for airmass relative in the future
    if (manual_control_sp_.enabled) {

        // check mode
        mode_changed = manual_control_sp_.type != last_mode;

        if (was_disabled || mode_changed) {
            // reset initial bearing and altitude

            // init roll to zero
            manual_control_sp_.roll = 0.0;

            // check if on ground, flying against strong wind, or just launched, otherwise set current course
            bool airsp_less_than_thres = x0_(IDX_X_V) < veh_parameters_.airsp_thres + 1.0;
            bool gsp_near_zero = x0_vel_(1)*x0_vel_(1) + x0_vel_(0)*x0_vel_(0) < 1.0;
            manual_control_sp_.bearing = (airsp_less_than_thres || gsp_near_zero) ? x0_(IDX_X_XI) : atan2(x0_vel_(1), x0_vel_(0));

            // set current altitude
            manual_control_sp_.alt = -x0_(IDX_X_POS+2);
            if (manual_control_sp_.type == ManualControlTypes::TERRAIN_ALTITUDE) {
                // consider terrain

                // get terrain altitude at current position
                double terr_alt = getTerrainAltitude(x0_(IDX_X_POS), x0_(IDX_X_POS+1), map_origin_north_, map_origin_east_,
                    map_height_, map_width_, map_resolution_, map_array_);

                // set relative altitude
                manual_control_sp_.rel_alt = std::max(manual_control_sp_.alt - terr_alt, 0.0); // must be above ground

                // set absolute altitude
                manual_control_sp_.alt = terr_alt + manual_control_sp_.rel_alt;
            }

            was_disabled = false;
        }
        else {
            // translate stick inputs and propagate manual control signals

            t_elapsed = ros::Time::now() - t_last;

            // propagate bearing from last bearing rate over elapsed time
            manual_control_sp_.bearing += manual_control_sp_.bearing_rate * std::min(t_elapsed.toSec(), node_parameters_.iteration_timestep);

            // propagate altitude from last unit ground velocity reference (if in altitude mode
            double delta_alt = -manual_control_sp_.airspeed * manual_control_sp_.unit_delta_alt_rate * std::min(t_elapsed.toSec(), node_parameters_.iteration_timestep);

            if (manual_control_sp_.type == ManualControlTypes::ALTITUDE) {
                // increment the absolute altitude
                manual_control_sp_.alt += delta_alt;
            }
            else if (manual_control_sp_.type == ManualControlTypes::TERRAIN_ALTITUDE) {
                // increment the relative altitude
                manual_control_sp_.rel_alt = std::max(manual_control_sp_.rel_alt + delta_alt, 0.0); // relative altitude must be positive
            }
        }

        // set new bearing -- INFO: why unit vel? why here? saves a horizon's length worth+ of sin and cos evals (these are held through the horizon)
        manual_control_sp_.unit_vel(0) = cos(manual_control_sp_.bearing);
        manual_control_sp_.unit_vel(1) = sin(manual_control_sp_.bearing);

        // throttle stick controls airspeed set point
        manual_control_sp_.airspeed = veh_parameters_.airsp_max * (constrain(msg->z, -1.0, 1.0)+1.0)*0.5 + veh_parameters_.airsp_min;

        const double one_minus_dz = 1.0 - MANUAL_CONTROL_DZ;

        // roll stick inputs
        double y_input;
        if (msg->y < 0.0) {
            y_input = constrain((msg->y + MANUAL_CONTROL_DZ) / one_minus_dz, -1.0, 0.0);
        }
        else {
            y_input = constrain((msg->y - MANUAL_CONTROL_DZ) / one_minus_dz, 0.0, 1.0);
        }

        // what to do with roll stick inputs
        if (manual_control_sp_.lat_input == ManualControlLatInputs::BEARING_RATE) {
            // roll stick controls rate of bearing
            manual_control_sp_.bearing_rate = y_input * manual_control_sp_.max_bearing_rate;
        }
        else { // default == ManualControlLatInputs::ROLL
            // roll stick controls (setpoint of) roll setpoint
            manual_control_sp_.roll = y_input * control_parameters_.roll_lim_rad;
        }

        // pitch stick controls flight path angle setpoint (ground relative)
        double unit_delta_z;
        if (msg->x < 0.0) {
            unit_delta_z = constrain((msg->x + MANUAL_CONTROL_DZ) / one_minus_dz, -1.0, 0.0)
                                * gl_.getUnitZAppMax();
        }
        else {
            unit_delta_z = constrain((msg->x - MANUAL_CONTROL_DZ) / one_minus_dz, 0.0, 1.0)
                                * gl_.getUnitZAppMax(); //TODO: again should have separate sink and climb params
        }
        if (manual_control_sp_.type == ManualControlTypes::ALTITUDE || manual_control_sp_.type == ManualControlTypes::TERRAIN_ALTITUDE) {
            manual_control_sp_.unit_vel(2) = 0.0; // this is calculated depending on the horizon node
            manual_control_sp_.unit_delta_alt_rate = unit_delta_z;
        }
        else {
            manual_control_sp_.unit_vel(2) = unit_delta_z;
        }

        // re-normalize decoupled (lat/lon) unit velocity setpoint
        renormalizeUnitVelocity(manual_control_sp_.unit_vel);
    }
    else {
        was_disabled = true;
    }

    t_last = ros::Time::now();
    last_mode = manual_control_sp_.type;
} // manCtrlCb

void NonlinearMPC::staticPresCb(const sensor_msgs::FluidPressure::ConstPtr& msg) // static pressure msg callback
{
    static_pressure_pa_ = msg->fluid_pressure;
} // staticPresCb

void NonlinearMPC::sysStatusCb(const mavros_msgs::State::ConstPtr& msg) // system status msg callback
{
    // this message comes at 1 Hz and tells us if the nmpc is currently controlling the aircraft
    std::string mode = msg->mode;
    if (mode != px4_mode_) {
        ROS_INFO_STREAM("Received Pixhawk Mode: " <<  mode);
        px4_mode_ = mode;
    }
    offboard_mode_ = (msg->mode == "OFFBOARD");
    px4_connected_ = msg->connected;
} // sysStatusCb

void NonlinearMPC::sysStatusExtCb(const mavros_msgs::ExtendedState::ConstPtr& msg) // system status msg callback
{
    landed_state_ = msg->landed_state;
} // sysStatusExtCb

void NonlinearMPC::tempCCb(const sensor_msgs::Temperature::ConstPtr& msg) // temperature msg callback
{
    temperature_c_ = msg->temperature;
} // tempCCb

void NonlinearMPC::windEstCb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg) // wind estimation msg callback
{
    // ENU->NED
    x0_wind_(0) = msg->twist.twist.linear.y;
    x0_wind_(1) = msg->twist.twist.linear.x;
    x0_wind_(2) = -msg->twist.twist.linear.z;
} // windEstCb

void NonlinearMPC::checkSubs()
{
    /* wait for home waypoint */
    while (home_lat_ < 0.1 && home_lon_ < 0.1) {
        ros::spinOnce();
        ROS_INFO ("check subs: waiting for home position");
        sleep(1.0);
    }
    home_pos_valid_ = true;
    ROS_ERROR("check subs: received home position");

    // XXX: should have checks (and/or timeouts) for the rest of the required state estimates

    return;
} // checkSubs

/*
    END CALLBACKS
*/

/*
    DYNAMIC RECONFIGURE
*/

void NonlinearMPC::parametersCallbackControl(const fw_ctrl::controlConfig &config, const uint32_t& level)
{
    // state weights
    w_(IDX_Y_VN) = config.q_vne;
    w_(IDX_Y_VE) = config.q_vne;
    w_(IDX_Y_VD) = config.q_vd;
    w_(IDX_Y_V) = config.q_v;
    w_(IDX_Y_PHI) = config.q_roll;
    w_(IDX_Y_THETA) = config.q_pitch;
    w_(IDX_Y_SOFT_AOA) = config.q_soft_aoa;
    w_(IDX_Y_SOFT_HAGL) = config.q_soft_hagl;
    w_(IDX_Y_SOFT_RTD) = config.q_soft_rtd;
    // control weights
    w_(IDX_Y_U_T) = config.r_throt;
    w_(IDX_Y_PHI_REF) = config.r_roll_ref;
    w_(IDX_Y_THETA_REF) = config.r_pitch_ref;

    // control bounds
    control_parameters_.roll_lim_rad  = config.roll_lim * DEG_TO_RAD;
    lb_(IDX_U_PHI_REF) = -config.roll_lim * DEG_TO_RAD;
    ub_(IDX_U_PHI_REF) = config.roll_lim * DEG_TO_RAD;
    lb_(IDX_U_THETA_REF) = config.pitch_lb * DEG_TO_RAD;
    ub_(IDX_U_THETA_REF) = config.pitch_ub * DEG_TO_RAD;

    // objective references
    control_parameters_.airsp_ref = config.airsp_ref;

    // terrain avoidance parameters
    control_parameters_.enable_terrain_feedback = config.enable_terrain_feedback;
    control_parameters_.tau_terr = config.tau_terr;

    // control reference parameters
    control_parameters_.use_ff_roll_ref = config.use_ff_roll_ref;
    control_parameters_.use_floating_ctrl_ref = config.use_floating_ctrl_ref;
    control_parameters_.tau_u = config.tau_u;
    control_parameters_.fixed_pitch_ref = config.fixed_pitch_ref * DEG_TO_RAD;
    control_parameters_.fixed_throt_ref = config.fixed_throt_ref;
} // parametersCallbackControl

void NonlinearMPC::parametersCallbackGuidance(const fw_ctrl::guidanceConfig &config, const uint32_t& level)
{
    gl_.setGuidanceType(GuidanceLogicTypes(config.guidance_sel)); // check if there is a cleaner way to do this with the enum already made in the cfg -- otherwise should probably check that this is in range);
    gl_.setUseOCCAsGuidance(config.use_occ_as_guidance);
    gl_.setFixedVertPosErrBound(config.fix_vert_pos_err_bnd);
    gl_.setVertPosErrBound(config.vert_pos_err_bnd);
    gl_.setLatTimeConstant(config.T_lat);
    gl_.setLonTimeConstant(config.T_lon);
    gl_.setFPAAppMax(config.gamma_app_max * DEG_TO_RAD);
} // parametersCallbackGuidance

void NonlinearMPC::parametersCallbackManualControl(const fw_ctrl::manual_controlConfig &config, const uint32_t& level)
{
    manual_control_sp_.enabled = config.en_man_ctrl;
    manual_control_sp_.type = ManualControlTypes(config.man_ctrl_type);
    if (manual_control_sp_.type == ManualControlTypes::TERRAIN_ALTITUDE && !(control_parameters_.enable_terrain_feedback && grid_map_valid_)) {
        manual_control_sp_.type = ManualControlTypes::ALTITUDE;
    }
    manual_control_sp_.lat_input = ManualControlLatInputs(config.man_ctrl_lat_input);
    manual_control_sp_.max_bearing_rate = config.max_bearing_rate * DEG_TO_RAD;
} // parametersCallbackManualControl

void NonlinearMPC::parametersCallbackOcclusionDetection(const fw_ctrl::occlusion_detectionConfig &config, const uint32_t& level)
{
    ray_casting_interval_ = constrain(config.ray_casting_interval, 1, ACADO_N+1);
    findNearestRayOrigins();
    len_occ_data_ = (ACADO_N+1 + ray_casting_interval_-1) / ray_casting_interval_;
    len_occ_window_ = constrain(config.len_occ_window, 1, len_occ_data_);
    setOcclusionWindows();
    len_occ_buffer_ = constrain(config.len_occ_buffer, 1, OcclusionDetector::LEN_BUFFER_MAX);
    rtd_node_interval_ = constrain(config.rtd_node_interval, 1, ACADO_N+1);
    surfel_radius_ = std::max(config.surfel_radius, 0.0);

    for (int i = 0; i < NUM_OCC_BUF; i++) {
        occ_[i].setDataLength(len_occ_data_);
        occ_[i].setBufferLength(len_occ_buffer_);
    }
} // parametersCallbackOcclusionDetection

void NonlinearMPC::parametersCallbackSoftConstraints(const fw_ctrl::soft_constraintsConfig &config, const uint32_t& level)
{
    // soft angle of attack parameters
    huber_aoa_p_.setConstraint(config.aoa_p*DEG_TO_RAD);
    huber_aoa_p_.setDelta(config.delta_aoa*DEG_TO_RAD);
    huber_aoa_p_.setCostAtOne(config.cost_aoa_1);
    huber_aoa_m_.setConstraint(config.aoa_m*DEG_TO_RAD);
    huber_aoa_m_.setDelta(config.delta_aoa*DEG_TO_RAD);
    huber_aoa_m_.setCostAtOne(config.cost_aoa_1);

    // soft height above ground level parameters
    huber_hagl_.setConstraint(config.hagl_offset); // nadir terrain offset [m]
    huber_hagl_.setDelta(config.delta_hagl); // nadir terrain buffer [m]
    huber_hagl_.setCostAtOne(config.cost_hagl_1);

    // soft radial terrain distance parameters
    huber_rtd_params_.constr_0 = config.rtd_offset; // radial terrain offset [m]
    huber_rtd_params_.delta_0 = config.rtd_delta; // radial terrain buffer (at zero relative velocity) [m]
    huber_rtd_params_.constr_gain = config.rtd_offset_gain;
    huber_rtd_params_.delta_gain = config.rtd_delta_gain;
} // parametersCallbackSoftConstraints

/*
    END DYNAMIC RECONFIGURE
*/

/*
    PUBLISHERS
*/

void NonlinearMPC::publishControls(const double u_T, const double phi_ref, const double theta_ref)
{
    // thrust sp
    mavros_msgs::Thrust thrust_sp;
    thrust_sp.header.stamp = ros::Time::now();
    thrust_sp.thrust = (float)constrain(mapNormalizedThrotToPX4Throt(u_T, acadoVariables.x[IDX_X_V], acadoVariables.x[IDX_X_THETA] - acadoVariables.x[IDX_X_GAMMA]), 0.0, 1.0);
    thrust_pub_.publish(thrust_sp);

    // attitude setpoint
    geometry_msgs::PoseStamped att_sp;
    att_sp.header.frame_id = "map";
    att_sp.header.stamp = ros::Time::now();

    tf::Quaternion q_ned;
    q_ned.setRPY(phi_ref, theta_ref, 0.0);

    // convert to ENU
    tf::Quaternion q_enu = ned_enu_q_ * q_ned * aircraft_baselink_q_; // XXX: again.. should probably just make a param to publish and receive ned vs enu on mavros (could even push upstream)

    geometry_msgs::Quaternion q_msg;
    quaternionTFToMsg(q_enu, q_msg);
    att_sp.pose.orientation = q_msg;
    att_sp_pub_.publish(att_sp);
} // publishControls

void NonlinearMPC::publishNMPCInfo(ros::Time t_iter_start, uint64_t t_ctrl, uint64_t t_preeval, uint64_t t_prep, uint64_t t_fb)
{
    fw_ctrl::NMPCInfo nmpc_info;

    // obctrl status
    nmpc_info.status = obctrl_status_;
    switch (obctrl_status_)
    {
        case OffboardControlStatus::PREPARATION_STEP_ERROR:
            nmpc_info.err_code = err_code_preparation_step_;
            break;

        case OffboardControlStatus::FEEDBACK_STEP_ERROR:
            nmpc_info.err_code = err_code_feedback_step_;
            break;

        default:
            nmpc_info.err_code = 0;
    }

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

void NonlinearMPC::publishNMPCStates()
{
    fw_ctrl::NMPCMeasurements nmpc_measurements;
    fw_ctrl::NMPCStates nmpc_states;
    fw_ctrl::NMPCControls nmpc_controls;
    fw_ctrl::NMPCOnlineData nmpc_online_data;
    fw_ctrl::NMPCObjRef nmpc_obj_ref;
    fw_ctrl::NMPCObjNRef nmpc_objN_ref;
    fw_ctrl::NMPCAuxOut nmpc_aux_output;

    /* measurements */
    nmpc_measurements.n = (float)x0_(0);
    nmpc_measurements.e = (float)x0_(1);
    nmpc_measurements.d = (float)x0_(2);
    nmpc_measurements.v = (float)x0_(3);
    nmpc_measurements.gamma = (float)x0_(4);
    nmpc_measurements.xi = (float)x0_(5);
    nmpc_measurements.phi = (float)x0_(6);
    nmpc_measurements.theta = (float)x0_(7);
    nmpc_measurements.n_prop = (float)x0_(8);

    /* state/control horizons */
    for (int i=0; i<ACADO_N; i++) {
        nmpc_states.n[i] = (float)acadoVariables.x[ACADO_NX * i + IDX_X_POS];
        nmpc_states.e[i] = (float)acadoVariables.x[ACADO_NX * i + IDX_X_POS+1];
        nmpc_states.d[i] = (float)acadoVariables.x[ACADO_NX * i + IDX_X_POS+2];
        nmpc_states.v[i] = (float)acadoVariables.x[ACADO_NX * i + IDX_X_V];
        nmpc_states.gamma[i]  = (float)acadoVariables.x[ACADO_NX * i + IDX_X_GAMMA];
        nmpc_states.xi[i] = (float)acadoVariables.x[ACADO_NX * i + IDX_X_XI];
        nmpc_states.phi[i] = (float)acadoVariables.x[ACADO_NX * i + IDX_X_PHI];
        nmpc_states.theta[i] = (float)acadoVariables.x[ACADO_NX * i + IDX_X_THETA];
        nmpc_states.n_prop[i] = (float)acadoVariables.x[ACADO_NX * i + IDX_X_NPROP];

        nmpc_controls.u_T[i] = (float)acadoVariables.u[ACADO_NU * i + IDX_U_U_T];
        nmpc_controls.phi_ref[i] = (float)acadoVariables.u[ACADO_NU * i + IDX_U_PHI_REF];
        nmpc_controls.theta_ref[i] = (float)acadoVariables.u[ACADO_NU * i + IDX_U_THETA_REF];
    }
    nmpc_states.n[ACADO_N] = (float)acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_POS];
    nmpc_states.e[ACADO_N] = (float)acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_POS+1];
    nmpc_states.d[ACADO_N] = (float)acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_POS+2];
    nmpc_states.v[ACADO_N] = (float)acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_V];
    nmpc_states.gamma[ACADO_N] = (float)acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_GAMMA];
    nmpc_states.xi[ACADO_N] = (float)acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_XI];
    nmpc_states.phi[ACADO_N] = (float)acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_PHI];
    nmpc_states.theta[ACADO_N] = (float)acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_THETA];
    nmpc_states.n_prop[ACADO_N] = (float)acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_NPROP];

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
    for (int i=0; i<ACADO_N+1; i++) { // all prior online data are held constant through the horizon
        nmpc_online_data.soft_aoa[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_SOFT_AOA];
        nmpc_online_data.jac_soft_aoa_0[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_AOA];
        nmpc_online_data.jac_soft_aoa_1[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_AOA+1];
        nmpc_online_data.soft_hagl[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_SOFT_HAGL];
        nmpc_online_data.jac_soft_hagl_0[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_HAGL];
        nmpc_online_data.jac_soft_hagl_1[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_HAGL+1];
        nmpc_online_data.jac_soft_hagl_2[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_HAGL+2];
        nmpc_online_data.jac_soft_hagl_3[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_HAGL+3];
        nmpc_online_data.soft_rtd[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_SOFT_RTD];
        nmpc_online_data.jac_soft_rtd_0[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_RTD];
        nmpc_online_data.jac_soft_rtd_1[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_RTD+1];
        nmpc_online_data.jac_soft_rtd_2[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_RTD+2];
        nmpc_online_data.jac_soft_rtd_3[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_RTD+3];
        nmpc_online_data.jac_soft_rtd_4[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_RTD+4];
        nmpc_online_data.jac_soft_rtd_5[i] = (float)acadoVariables.od[ACADO_NOD * i + IDX_OD_JAC_SOFT_RTD+5];
    }

    /* objective references */
    for (int i=0; i<ACADO_N; i++) {
        nmpc_obj_ref.vn[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_VN];
        nmpc_obj_ref.ve[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_VE];
        nmpc_obj_ref.vd[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_VD];
        nmpc_obj_ref.v[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_V];
        nmpc_obj_ref.phi[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_PHI];
        nmpc_obj_ref.theta[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_THETA];
        nmpc_obj_ref.soft_aoa[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_SOFT_AOA];
        nmpc_obj_ref.soft_hagl[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_SOFT_HAGL];
        nmpc_obj_ref.soft_rtd[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_SOFT_RTD];

        nmpc_obj_ref.u_T[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_U_T];
        nmpc_obj_ref.phi_ref[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_PHI_REF];
        nmpc_obj_ref.theta_ref[i] = (float)acadoVariables.y[ACADO_NY * i + IDX_Y_THETA_REF];
    }
    nmpc_objN_ref.vn = (float)acadoVariables.yN[IDX_Y_VN];
    nmpc_objN_ref.ve = (float)acadoVariables.yN[IDX_Y_VE];
    nmpc_objN_ref.vd = (float)acadoVariables.yN[IDX_Y_VD];
    nmpc_objN_ref.v = (float)acadoVariables.yN[IDX_Y_V];
    nmpc_objN_ref.phi = (float)acadoVariables.yN[IDX_Y_PHI];
    nmpc_objN_ref.theta = (float)acadoVariables.yN[IDX_Y_THETA];
    nmpc_objN_ref.soft_aoa = (float)acadoVariables.yN[IDX_Y_SOFT_AOA];
    nmpc_objN_ref.soft_hagl = (float)acadoVariables.yN[IDX_Y_SOFT_HAGL];
    nmpc_objN_ref.soft_rtd = (float)acadoVariables.yN[IDX_Y_SOFT_RTD];

    /* auxiliary outputs */

    // arbitrary init
    Eigen::Vector3d unit_ground_vel_sp(1.0, 0.0, 0.0);
    double err_lat = 0.0;
    double err_lon = 0.0;
    double unit_err_lat = 0.0;

    // lateral-directional ground speed
    const double ground_sp_lat = x0_vel_.segment(0,2).norm();

    // get the terrain altitude at the current position (only used in terrain following mode)
    const double terrain_alt = getTerrainAltitude(x0_(IDX_X_POS), x0_(IDX_X_POS+1), map_origin_north_, map_origin_east_,
        map_height_, map_width_, map_resolution_, map_array_);

    // calculate the unit velocity reference for the current measured state
    gl_.calculateUnitVelocityReference(unit_ground_vel_sp, err_lat, err_lon, unit_err_lat,
        x0_pos_, x0_vel_, ground_sp_lat, terrain_alt, manual_control_sp_, path_sp_);

    if (gl_.getUseOCCAsGuidance()) {
        // steer current guidance vector with terrain cost jacobians
        gl_.augmentTerrainCostToGuidance(unit_ground_vel_sp, od_.block(IDX_OD_SOFT_RTD, 0, 3, 1).normalized(), inv_prio_rtd_(0));
    }

    // at current measured states
    double ground_sp = x0_vel_.norm();
    const double inv_ground_sp = (ground_sp < 0.01) ? 100.0 : 1.0 / ground_sp;
    nmpc_aux_output.path_error_lat = err_lat;
    nmpc_aux_output.path_error_lon = err_lon;
    nmpc_aux_output.err_v_n_unit = unit_ground_vel_sp(0) - x0_vel_(0) * inv_ground_sp; // unit northing ground velocity error [~]
    nmpc_aux_output.err_v_e_unit = unit_ground_vel_sp(1) - x0_vel_(1) * inv_ground_sp; // unit easting ground velocity error [~]
    nmpc_aux_output.err_v_d_unit = unit_ground_vel_sp(2) - x0_vel_(2) * inv_ground_sp; // unit downing ground velocity error [~]

    /* publish */
    nmpc_meas_pub_.publish(nmpc_measurements);
    nmpc_states_pub_.publish(nmpc_states);
    nmpc_controls_pub_.publish(nmpc_controls);
    nmpc_online_data_pub_.publish(nmpc_online_data);
    nmpc_obj_ref_pub_.publish(nmpc_obj_ref);
    nmpc_objN_ref_pub_.publish(nmpc_objN_ref);
    nmpc_aux_out_pub_.publish(nmpc_aux_output);
} // publishNMPCStates

void NonlinearMPC::publishNMPCVisualizations()
{
    // TODO: these visualization functions should be moved to a separate node

    // publish msgs for rviz visualization

    // path references (converted to robotic frame)
    if (path_sp_.type == PathTypes::LOITER) {
        nav_msgs::Path guidance_path;
        guidance_path.header.frame_id = "map";
        int num_pts = 50;
        guidance_path.poses = std::vector<geometry_msgs::PoseStamped>(num_pts+1);
        for (int i=0; i<num_pts+1; i++) {
            guidance_path.poses[i].pose.position.x = path_sp_.pos(1) + fabs(path_sp_.signed_radius)*sin(double(i)/double(num_pts)*M_PI*2.0);
            guidance_path.poses[i].pose.position.y = path_sp_.pos(0) + fabs(path_sp_.signed_radius)*cos(double(i)/double(num_pts)*M_PI*2.0);
            guidance_path.poses[i].pose.position.z = -path_sp_.pos(2);
        }
        nmpc_guidance_path_pub_.publish(guidance_path);
    }

    // predicted poses (converted to robotic frame)
    nav_msgs::Path nmpc_traj_pred;
    nmpc_traj_pred.header.frame_id = "map";
    nmpc_traj_pred.poses = std::vector<geometry_msgs::PoseStamped>(ACADO_N+1);
    for (int i=0; i<ACADO_N+1; i++) {
        // NED->ENU
        nmpc_traj_pred.poses[i].pose.position.x = acadoVariables.x[ACADO_NX * i + IDX_X_POS+1];
        nmpc_traj_pred.poses[i].pose.position.y = acadoVariables.x[ACADO_NX * i + IDX_X_POS];
        nmpc_traj_pred.poses[i].pose.position.z = -acadoVariables.x[ACADO_NX * i + IDX_X_POS+2];
        // NED->ENU
        tf::Quaternion q_ned;
        q_ned.setRPY(acadoVariables.x[ACADO_NX * i + IDX_X_PHI], acadoVariables.x[ACADO_NX * i + IDX_X_THETA], acadoVariables.x[ACADO_NX * i + IDX_X_XI]);
        tf::Quaternion q_enu = ned_enu_q_ * q_ned * aircraft_baselink_q_;
        quaternionTFToMsg(q_enu, nmpc_traj_pred.poses[i].pose.orientation);
    }
    nmpc_traj_pred_pub_.publish(nmpc_traj_pred);

    // rays, normals, and planes
    if (control_parameters_.enable_terrain_feedback && grid_map_valid_) {
        auto normals_msg = boost::make_shared<visualization_msgs::Marker>();
        auto planes_msg = boost::make_shared<visualization_msgs::Marker>();
        auto rays_msg = boost::make_shared<visualization_msgs::Marker>();
        bool occlusions_to_visualize = populateOcclusionLists(rays_msg, normals_msg, planes_msg);
        if (occlusions_to_visualize) {
            nmpc_occ_normals_pub_.publish(normals_msg);
            nmpc_occ_planes_pub_.publish(planes_msg);
        }
        nmpc_occ_rays_pub_.publish(rays_msg);
    }

    // unit ground velocity setpoints
    visualization_msgs::MarkerArray unit_gnd_vel_ref_msg;
    int marker_counter = 0;
    for (int i=0; i<ACADO_N; i++) {
        visualization_msgs::Marker unit_gnd_vel_ref;
        unit_gnd_vel_ref.header.frame_id = "map";
        unit_gnd_vel_ref.header.stamp = ros::Time();
        unit_gnd_vel_ref.ns = "unit_gnd_vel_ref";
        unit_gnd_vel_ref.id = ++marker_counter;
        unit_gnd_vel_ref.type = visualization_msgs::Marker::ARROW;
        unit_gnd_vel_ref.action = visualization_msgs::Marker::ADD;
        // NED->ENU
        unit_gnd_vel_ref.pose.position.x = acadoVariables.x[ACADO_NX * i + IDX_X_POS+1];
        unit_gnd_vel_ref.pose.position.y = acadoVariables.x[ACADO_NX * i + IDX_X_POS];
        unit_gnd_vel_ref.pose.position.z = -acadoVariables.x[ACADO_NX * i + IDX_X_POS+2];
        Eigen::Vector3d v(acadoVariables.y[ACADO_NY * i + IDX_Y_VE], acadoVariables.y[ACADO_NY * i + IDX_Y_VN], -acadoVariables.y[ACADO_NY * i + IDX_Y_VD]); // ENU
        Eigen::Quaterniond q_enu_to_arrow;
        q_enu_to_arrow.setFromTwoVectors(Eigen::Vector3d::UnitX(), v);
        tf::quaternionEigenToMsg(q_enu_to_arrow, unit_gnd_vel_ref.pose.orientation);
        unit_gnd_vel_ref.scale.x = 5.0; // marker length
        unit_gnd_vel_ref.scale.y = 0.5; // arrow width
        unit_gnd_vel_ref.scale.z = 1.0; // arrow height
        unit_gnd_vel_ref.color.a = 1.0; // alpha
        unit_gnd_vel_ref.color.r = 0.0;
        unit_gnd_vel_ref.color.g = 0.8;
        unit_gnd_vel_ref.color.b = 0.0;
        unit_gnd_vel_ref.lifetime = ros::Duration(node_parameters_.iteration_timestep);

        // push back unit ground vel marker
        unit_gnd_vel_ref_msg.markers.push_back(unit_gnd_vel_ref);
    }
    unit_gnd_vel_ref_pub_.publish(unit_gnd_vel_ref_msg);
} // publishNMPCVisualizations

bool NonlinearMPC::populateOcclusionLists(visualization_msgs::Marker::Ptr rays_msg,
    visualization_msgs::Marker::Ptr normals_msg, visualization_msgs::Marker::Ptr planes_msg)
{
    bool occlusions_to_visualize = false;

    // rays
    rays_msg->header.frame_id = "map";
    rays_msg->header.stamp = ros::Time();
    rays_msg->ns = "rays";
    rays_msg->id = 0;
    rays_msg->type = visualization_msgs::Marker::LINE_LIST;
    rays_msg->action = visualization_msgs::Marker::ADD;
    rays_msg->pose.orientation.w = 1.0;
    rays_msg->scale.x = 0.5; // line width
    rays_msg->color.a = 1.0; // alpha
    rays_msg->color.r = 1.0;
    rays_msg->color.g = 0.0;
    rays_msg->color.b = 0.0;
    rays_msg->lifetime = ros::Duration(node_parameters_.nmpc_iteration_rate);

    // keep them alive as long as they are stored in the buffer
    ros::Duration lifetime(node_parameters_.iteration_timestep * len_occ_buffer_);

    // occlusion normals
    normals_msg->header.frame_id = "map";
    normals_msg->header.stamp = ros::Time();
    normals_msg->ns = "surface_normals";
    normals_msg->id = 1;
    normals_msg->type = visualization_msgs::Marker::LINE_LIST;
    normals_msg->action = visualization_msgs::Marker::ADD;
    normals_msg->pose.orientation.w = 1.0;
    normals_msg->scale.x = 1.0; // line width
    normals_msg->color.a = 1.0; // alpha
    normals_msg->color.r = 1.0;
    normals_msg->color.g = 0.0;
    normals_msg->color.b = 0.0;
    normals_msg->lifetime = lifetime;
    double len_normal = 5.0;

    // occlusion plane
    planes_msg->header.frame_id = "map";
    planes_msg->header.stamp = ros::Time();
    planes_msg->ns = "surface_planes";
    planes_msg->id = 2;
    planes_msg->type = visualization_msgs::Marker::LINE_LIST;
    planes_msg->action = visualization_msgs::Marker::ADD;
    planes_msg->pose.orientation.w = 1.0;
    planes_msg->scale.x = std::max(2.0 * surfel_radius_, 2.0); // line width
    planes_msg->color.a = 1.0; // alpha
    planes_msg->color.r = 1.0;
    planes_msg->color.g = 0.0;
    planes_msg->color.b = 0.0;
    planes_msg->lifetime = lifetime;
    double len_plane = 1.0;

    // for all buffers
    for (int j_buf = 0; j_buf < NUM_OCC_BUF; j_buf++) {

        // for length of data in current buffer head
        for (int i_data = 0; i_data < len_occ_data_; i_data++) {

            geometry_msgs::Point p;
            p.x = occ_[j_buf].ray_list_[i_data][IDX_OCC_POS];
            p.y = occ_[j_buf].ray_list_[i_data][IDX_OCC_POS+1];
            p.z = occ_[j_buf].ray_list_[i_data][IDX_OCC_POS+2];

            rays_msg->points.push_back(p);

            p.x = occ_[j_buf].ray_list_[i_data][IDX_OCC_POS] + occ_[j_buf].ray_list_[i_data][IDX_OCC_NORMAL] * occ_[j_buf].ray_list_[i_data][IDX_OCC_RAY_LEN];
            p.y = occ_[j_buf].ray_list_[i_data][IDX_OCC_POS+1] + occ_[j_buf].ray_list_[i_data][IDX_OCC_NORMAL+1] * occ_[j_buf].ray_list_[i_data][IDX_OCC_RAY_LEN];
            p.z = occ_[j_buf].ray_list_[i_data][IDX_OCC_POS+2] + occ_[j_buf].ray_list_[i_data][IDX_OCC_NORMAL+2] * occ_[j_buf].ray_list_[i_data][IDX_OCC_RAY_LEN];

            rays_msg->points.push_back(p);

        } // end i_data loop

        // check if we had any detections
        if (occ_[j_buf].getDetectionCountAtHead()) {

            // mark that we have something to visualize
            occlusions_to_visualize = true;

            // get head for current buffer
            int buffer_head = occ_[j_buf].getBufferHead();

            // for length of data in current buffer head
            for (int i_data = 0; i_data < len_occ_data_; i_data++) {

                // check for a detection
                if (occ_[j_buf].detections_[buffer_head][i_data]) {

                    // surfel normals (lines)

                    geometry_msgs::Point p;
                    p.x = occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_POS+1];
                    p.y = occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_POS];
                    p.z = -occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_POS+2];

                    normals_msg->points.push_back(p);
                    planes_msg->points.push_back(p); // same start point for plane

                    p.x = occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_POS+1] + occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_NORMAL+1] * len_normal;
                    p.y = occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_POS] + occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_NORMAL] * len_normal;
                    p.z = -occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_POS+2] - occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_NORMAL+2] * len_normal;

                    normals_msg->points.push_back(p);

                    // surfel plane

                    p.x = occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_POS+1] + occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_NORMAL+1] * len_plane;
                    p.y = occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_POS] + occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_NORMAL] * len_plane;
                    p.z = -occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_POS+2] - occ_[j_buf].attributes_[buffer_head][i_data][IDX_OCC_NORMAL+2] * len_plane;

                    planes_msg->points.push_back(p);

                } // endif check for detection at i_data
            } // end i_data loop
        } // endif check for any detections in buffer
    } // end j_buf loop

    return occlusions_to_visualize;
} // populateOcclusionLists

/*
    END PUBLISHERS
*/

/*
    AIRCRAFT STATE CONVERSIONS / CALCULATIONS
*/

void NonlinearMPC::calculateSpeedStatesHorizon()
{
    /*
        calculates the speed states at every node
    */

    for (int i = 0; i < ACADO_N; i++) {
        calculateSpeedStates(&spds_[i].air_vel[0], &spds_[i].ground_vel[0],
            spds_[i].ground_sp_sq, spds_[i].ground_sp, spds_[i].inv_ground_sp, &spds_[i].unit_ground_vel[0],
            spds_[i].ground_sp_lat_sq, spds_[i].ground_sp_lat, &spds_[i].unit_ground_vel_lat[0],
            acadoVariables.x + (i * ACADO_NX + IDX_X_V), x0_wind_.data());
    }
} // calculateSpeedStatesHorizon

double NonlinearMPC::flapsToRad(const double flaps_normalized)
{
    // for now, assuming that a full normalized signal corresponds to full actuation .. need to check this with px4 convention
    return veh_parameters_.flaps_lim_rad * constrain(flaps_normalized, -1.0, 1.0);
} // flapsToRad

double NonlinearMPC::mapPropSpeedToNormalizedThrot(const double n_prop, const double airsp, const double aoa)
{
    // prop speed input (converted from throttle input considering inflow and zero thrusting conditions)
    const double vp = airsp * cos(aoa - prop_parameters_.epsilon_T_rad); // inflow at propeller
    const double sig_vp = constrain((vp - prop_parameters_.vp_min) / (prop_parameters_.vp_max - prop_parameters_.vp_min), 0.0, 1.0); // prop inflow linear interpolater
    const double n_min = prop_parameters_.n_T0_vmin * (1.0 - sig_vp) + prop_parameters_.n_T0_vmax * sig_vp; // airspeed dependent min prop speed for linear interpolation
    return constrain((n_prop - n_min)/(prop_parameters_.n_max - n_min), 0.0, 1.0); // interpolate normalized throttle
} // mapPropSpeedToNormalizedThrot

double NonlinearMPC::mapPX4ThrotToNormalizedThrot(const double px4_throt, const double airsp, const double aoa)
{
    // px4_throt [0,1] -> propeller speed in static condition [rps] (linear fit)
    double n_prop_static = constrain(px4_throt * prop_parameters_.n_delta + prop_parameters_.n_0, 0.0, prop_parameters_.n_max);

    // n_prop_static -> normalized (with airspeed and zero thrust bounds) throttle input used in NMPC
    return mapPropSpeedToNormalizedThrot(n_prop_static, airsp, aoa);
} // mapPX4ThrotToNormalizedThrot

double NonlinearMPC::mapNormalizedThrotToPropSpeed(const double throt, const double airsp, const double aoa)
{
    // prop speed input (converted from throttle input considering inflow and zero thrusting conditions)
    const double vp = airsp * cos(aoa - prop_parameters_.epsilon_T_rad); // inflow at propeller
    const double sig_vp = constrain((vp - prop_parameters_.vp_min) / (prop_parameters_.vp_max - prop_parameters_.vp_min), 0.0, 1.0); // prop inflow linear interpolater
    return (prop_parameters_.n_T0_vmin + throt * (prop_parameters_.n_max - prop_parameters_.n_T0_vmin)) * (1.0 - sig_vp) + (prop_parameters_.n_T0_vmax + throt * (prop_parameters_.n_max - prop_parameters_.n_T0_vmax)) * sig_vp;
} // mapNormalizedThrotToPropSpeed

double NonlinearMPC::mapNormalizedThrotToPX4Throt(const double throt, const double airsp, const double aoa)
{
    double n_prop = mapNormalizedThrotToPropSpeed(throt, airsp, aoa);
    return (n_prop - prop_parameters_.n_0) / prop_parameters_.n_delta; // NOTE: this does not include dead-zone or true zero (prop may slowly spin at zero input)
} // mapNormalizedThrotToPX4Throt

void NonlinearMPC::propagateVirtPropSpeed(const double throt, const double airsp, const double aoa)
{
    n_prop_virt_ += (mapNormalizedThrotToPropSpeed(throt, airsp, aoa) - n_prop_virt_) / model_parameters_.tau_n_prop * node_parameters_.iteration_timestep;
} // propagateVirtPropSpeed

/*
    END AIRCRAFT STATE CONVERSIONS / CALCULATIONS
*/

/*
    OBJECTIVE PRE-EVALUATION FUNCTIONS
*/

void NonlinearMPC::detectOcclusions()
{
    if (control_parameters_.enable_terrain_feedback && grid_map_valid_) {

        if (terrain_was_disabled_) {
            // clear any old buffer
            for (int i = 0; i < NUM_OCC_BUF; i++) {
                occ_[i].clear();
            }
            terrain_was_disabled_ = false;
        }

        // rotate the buffer heads
        for (int i = 0; i < NUM_OCC_BUF; i++) {
            occ_[i].rotate();
        }

        // update ray list
        const double ray_length_0 = huber_rtd_params_.constr_0 + huber_rtd_params_.delta_0 + map_resolution_;
        for (int i = 0; i < len_occ_data_; i++) {

            // MPC node index
            int k_node = i*ray_casting_interval_;

            // they all get the same ray origins
            for (int j = 0; j < NUM_OCC_BUF; j++) {
                occ_[j].ray_list_[i][IDX_OCC_POS] = acadoVariables.x[ACADO_NX*k_node + IDX_X_POS+1]; // E
                occ_[j].ray_list_[i][IDX_OCC_POS+1] = acadoVariables.x[ACADO_NX*k_node + IDX_X_POS]; // N
                occ_[j].ray_list_[i][IDX_OCC_POS+2] = -acadoVariables.x[ACADO_NX*k_node + IDX_X_POS+2]; // U
            }

            // forward ray direction
            occ_[IDX_OCC_FWD].ray_list_[i][IDX_OCC_NORMAL] = spds_[k_node].unit_ground_vel[1]; // E
            occ_[IDX_OCC_FWD].ray_list_[i][IDX_OCC_NORMAL+1] = spds_[k_node].unit_ground_vel[0]; // N
            occ_[IDX_OCC_FWD].ray_list_[i][IDX_OCC_NORMAL+2] = -spds_[k_node].unit_ground_vel[2]; // U
            // forward ray length
            occ_[IDX_OCC_FWD].ray_list_[i][IDX_OCC_RAY_LEN]
                = ext_obj_.calculateRTDConstraint(spds_[k_node].ground_sp_sq, huber_rtd_params_.constr_0, huber_rtd_params_.constr_scaler)
                + ext_obj_.calculateRTDDelta(spds_[k_node].ground_sp_sq, huber_rtd_params_.delta_0, huber_rtd_params_.delta_scaler)
                + map_resolution_;

            // right ray direction
            occ_[IDX_OCC_RIGHT].ray_list_[i][IDX_OCC_NORMAL] = spds_[k_node].unit_ground_vel_lat[0]; // N -> E
            occ_[IDX_OCC_RIGHT].ray_list_[i][IDX_OCC_NORMAL+1] = -spds_[k_node].unit_ground_vel_lat[1]; // -E -> N
            occ_[IDX_OCC_RIGHT].ray_list_[i][IDX_OCC_NORMAL+2] = 0.0;
            // forward ray length
            occ_[IDX_OCC_RIGHT].ray_list_[i][IDX_OCC_RAY_LEN] = ray_length_0;

            // left ray direction
            occ_[IDX_OCC_LEFT].ray_list_[i][IDX_OCC_NORMAL] = -spds_[k_node].unit_ground_vel_lat[0]; // -N -> E
            occ_[IDX_OCC_LEFT].ray_list_[i][IDX_OCC_NORMAL+1] = spds_[k_node].unit_ground_vel_lat[1]; // E -> N
            occ_[IDX_OCC_LEFT].ray_list_[i][IDX_OCC_NORMAL+2] = 0.0;
            // forward ray length
            occ_[IDX_OCC_LEFT].ray_list_[i][IDX_OCC_RAY_LEN] = ray_length_0;
        }

        // cast the rays
        for (int i = 0; i < NUM_OCC_BUF; i++) {
            occ_[i].castRays(map_origin_north_, map_origin_east_, map_height_, map_width_, map_resolution_, map_array_);
        }
    }
    else {
        terrain_was_disabled_ = true;
    }
} // detectOcclusions

void NonlinearMPC::evaluateAOAObjective()
{
    // allocate
    double aoa, jac_aoa[LEN_JAC_SOFT_AOA];
    double huber_cost_m, huber_jac_m[LEN_JAC_SOFT_AOA], inv_prio_m;
    double huber_cost_p, huber_jac_p[LEN_JAC_SOFT_AOA], inv_prio_p;

    // for all nodes
    for (int i_node = 0; i_node < ACADO_N+1; i_node++) {

        // angle of attack
        aoa = acadoVariables.x[ACADO_NX * i_node + IDX_X_THETA] - acadoVariables.x[ACADO_NX * i_node + IDX_X_GAMMA];

        // angle of attack jacobian
        ext_obj_.jacobianAOA(jac_aoa);

        // lower constraint
        huber_aoa_m_.costAndJacobian(huber_cost_m, huber_jac_m, inv_prio_m, aoa, jac_aoa, LEN_JAC_SOFT_AOA);

        // upper constraint
        huber_aoa_p_.costAndJacobian(huber_cost_p, huber_jac_p, inv_prio_p, aoa, jac_aoa, LEN_JAC_SOFT_AOA);

        // cost
        od_(IDX_OD_SOFT_AOA, i_node) = huber_cost_m + huber_cost_p;

        // inverse priority
        inv_prio_aoa_(i_node) = constrain(inv_prio_m + inv_prio_p, 0.0, 1.0);

        // jacobian
        for (int ii = 0; ii < LEN_JAC_SOFT_AOA; ii++) {
            od_(IDX_OD_JAC_SOFT_AOA+ii, i_node) = huber_jac_m[ii] + huber_jac_p[ii];
        }
    } // end i_node loop
} // evaluateAOAObjective

void NonlinearMPC::evaluateHAGLObjective()
{
    if (control_parameters_.enable_terrain_feedback && grid_map_valid_) {

        // allocate
        double hagl, jac_hagl[LEN_JAC_SOFT_HAGL];
        double huber_cost, huber_jac[LEN_JAC_SOFT_HAGL], inv_prio;
        double corners[4], h12, h34, unit_east;

        // for all nodes
        for (int i_node = 0; i_node < ACADO_N+1; i_node++) {

            // terrain altitude from grid map
            terrain_alt_horizon_(i_node) = getTerrainAltitude(corners, h12, h34, unit_east,
                acadoVariables.x[ACADO_NX * i_node + IDX_X_POS], acadoVariables.x[ACADO_NX * i_node + IDX_X_POS+1],
                map_origin_north_, map_origin_east_, map_height_, map_width_, map_resolution_, map_array_);

            // height above ground level
            hagl = -acadoVariables.x[ACADO_NX * i_node + IDX_X_POS+2] - terrain_alt_horizon_(i_node);

            // hagl jacobian
            ext_obj_.jacobianHAGL(jac_hagl, corners, h12, h34, unit_east, map_resolution_);

            // lower constraint
            huber_hagl_.costAndJacobian(huber_cost, huber_jac, inv_prio, hagl, jac_hagl, LEN_JAC_SOFT_HAGL);

            // cost
            od_(IDX_OD_SOFT_HAGL, i_node) = huber_cost;

            // inverse priority
            inv_prio_hagl_(i_node) = inv_prio;

            // jacobian
            for (int ii = 0; ii < LEN_JAC_SOFT_HAGL; ii++) {
                od_(IDX_OD_JAC_SOFT_HAGL+ii, i_node) = huber_jac[ii];
            }
        } // end i_node loop
    }
    else {
        // zero
        od_.block(IDX_OD_SOFT_HAGL, 0, 1, ACADO_N+1);
        od_.block(IDX_OD_JAC_SOFT_HAGL, 0, LEN_JAC_SOFT_HAGL, ACADO_N+1);
        inv_prio_hagl_.setOnes();
    }
} // evaluateHAGLObjective

void NonlinearMPC::evaluateRTDObjective()
{
    if (control_parameters_.enable_terrain_feedback && grid_map_valid_) {

        // for all nodes we wish to calculate costs / jacobians
        for (int i_node = 0; i_node < ACADO_N+1; i_node += rtd_node_interval_) {

            // initialize
            double rtd_cost = 0.0;
            double rtd_cost_sum = 0.0;
            double rtd_inv_prio = 1.0;
            double rtd_jac_sum[LEN_JAC_SOFT_RTD];
            for (int ii = 0; ii < LEN_JAC_SOFT_RTD; ii++) {
                rtd_jac_sum[ii] = 0.0;
            }

            // get node states / intermediate calcs
            Eigen::Vector3d veh_pos(acadoVariables.x[ACADO_NX*i_node + IDX_X_POS], acadoVariables.x[ACADO_NX*i_node + IDX_X_POS+1], acadoVariables.x[ACADO_NX*i_node + IDX_X_POS+2]); // NED
            Eigen::Vector3d ground_vel(spds_[i_node].ground_vel[0], spds_[i_node].ground_vel[1], spds_[i_node].ground_vel[2]); // NED
            Eigen::Matrix<double, 3, 3> J_vel;
            ext_obj_.calculateVelocityJacobianMatrix(J_vel, acadoVariables.x[ACADO_NX*i_node + IDX_X_V],
                acadoVariables.x[ACADO_NX*i_node + IDX_X_GAMMA], acadoVariables.x[ACADO_NX*i_node + IDX_X_XI]);

            // NOTE: WE ARE ASSUMING ALL BUFFERS ARE SET TO EQUIVALENT PARAMETERS
            // >> the following iterators would need to be adapted if one wished to de-synchronize these values
            // (the vectorization of the occ_ struct is done here only for organizational purposes)

            // occlusion window starting index for current mpc node (considering nearest ray origin)
            int st_occ_win = occ_window_start_(nearest_ray_origin_(i_node));

            // for all buffers
            for (int j_buf = 0; j_buf < NUM_OCC_BUF; j_buf++) {

                // for all occlusions in occlusion window
                for (int k_data = st_occ_win; k_data < st_occ_win + len_occ_window_; k_data++) {

                    // for all occlusions in buffer history
                    for (int l_hist = 0; l_hist < len_occ_buffer_; l_hist++) {

                        // initialize
                        double rtd_cost_local = 0.0;
                        double rtd_inv_prio_local = 1.0;
                        double rtd_jac_local[LEN_JAC_SOFT_RTD];
                        for (int ii = 0; ii < LEN_JAC_SOFT_RTD; ii++) {
                            rtd_jac_local[ii] = 0.0;
                        }

                        // check if we had a detection
                        if (occ_[j_buf].detections_[l_hist][k_data]) {

                            // get occlusion position and normal from buffer
                            Eigen::Vector3d occ_pos(occ_[j_buf].attributes_[l_hist][k_data][IDX_OCC_POS],
                                occ_[j_buf].attributes_[l_hist][k_data][IDX_OCC_POS+1],
                                occ_[j_buf].attributes_[l_hist][k_data][IDX_OCC_POS+2]);
                            Eigen::Vector3d occ_normal(occ_[j_buf].attributes_[l_hist][k_data][IDX_OCC_NORMAL],
                                occ_[j_buf].attributes_[l_hist][k_data][IDX_OCC_NORMAL+1],
                                occ_[j_buf].attributes_[l_hist][k_data][IDX_OCC_NORMAL+2]);

                            // calculate cost / jacobian
                            ext_obj_.calculateRTDCostAndJacobian(rtd_cost_local, rtd_jac_local, LEN_JAC_SOFT_RTD, rtd_inv_prio_local,
                                occ_pos, occ_normal, veh_pos, ground_vel, J_vel, surfel_radius_,
                                huber_rtd_params_.constr_0, huber_rtd_params_.constr_scaler,
                                huber_rtd_params_.delta_0, huber_rtd_params_.delta_scaler);

                            // add to weighted jacobian sum
                            for (int ii = 0; ii < LEN_JAC_SOFT_RTD; ii++) {
                                rtd_jac_sum[ii] += rtd_cost_local * rtd_jac_local[ii];
                            }

                            // sum the costs used for weighting
                            rtd_cost_sum += rtd_cost_local;

                            // update maximum cost / priority
                            if (rtd_cost_local > rtd_cost) {
                                rtd_cost = rtd_cost_local;
                                rtd_inv_prio = rtd_inv_prio_local;
                            }
                        }
                    } // end l_hist loop
                } // end k_data loop
            } // end j_buf loop

            // set node cost
            od_(IDX_OD_SOFT_RTD, i_node) = rtd_cost;

            // and corresponding inverse priority
            inv_prio_rtd_(i_node) = rtd_inv_prio;

            // calculate weighted average jacobian for node
            double inv_cost_sum = 0.0;
            if (rtd_cost_sum > 0.0001) inv_cost_sum = 1.0 / rtd_cost_sum;
            for (int ii = 0; ii < LEN_JAC_SOFT_RTD; ii++) {
                od_(IDX_OD_JAC_SOFT_RTD+ii, i_node) = rtd_jac_sum[ii] * inv_cost_sum;
            }
        } // end i_node loop

        // !! IF we are skipping nodes...
        if (rtd_node_interval_ > 1) {
            // linearly interpolate costs / jacobians to intermediate nodes
            for (int i = rtd_node_interval_; i < ACADO_N+1; i += rtd_node_interval_) {
                int i_last = i - rtd_node_interval_;

                // slope w.r.t. index -- we are using constant time discretization in horizon
                const double inv_di = 1.0 / (i - i_last);

                // slope of the cost
                const double slope_cost = (od_(IDX_OD_SOFT_RTD, i) - od_(IDX_OD_SOFT_RTD, i_last)) * inv_di;

                // slope of the jacobians
                double slope_jac[LEN_JAC_SOFT_RTD];
                for (int ii = 0; ii < LEN_JAC_SOFT_RTD; ii++) {
                    slope_jac[ii] = (od_(IDX_OD_JAC_SOFT_RTD+ii, i) - od_(IDX_OD_JAC_SOFT_RTD+ii, i_last)) * inv_di;
                }

                // slope of the inverse priority
                const double slope_inv_prio = (inv_prio_rtd_(i) - inv_prio_rtd_(i_last)) * inv_di;

                // interpolate intermediate costs / jacobians
                for (int j = i_last + 1; j < i; j++) {

                    // cost
                    od_(IDX_OD_SOFT_RTD, j) = slope_cost * (j - i_last) + od_(IDX_OD_SOFT_RTD, i_last);

                    // jacobians
                    for (int ii = 0; ii < LEN_JAC_SOFT_RTD; ii++) {
                        od_(IDX_OD_JAC_SOFT_RTD+ii, j) = slope_jac[ii] * (j - i_last) + od_(IDX_OD_JAC_SOFT_RTD+ii, i_last);
                    }

                    // inverse priority
                    inv_prio_rtd_(j) = slope_inv_prio * (j - i_last) + inv_prio_rtd_(i_last);
                }
            }
            const int remaining_nodes = (ACADO_N+1 - 1) % rtd_node_interval_;
            if (remaining_nodes) {
                int i_remaining = ACADO_N+1 - remaining_nodes; // start of the remaining node indices
                int i_last = i_remaining - 1; // last node index with calculated cost / jacobian

                // hold costs / jacobians / inverse priorities constant through remaining nodes
                for (int i = i_remaining; i < ACADO_N+1; i++) {
                    // cost
                    od_(IDX_OD_SOFT_RTD, i) = od_(IDX_OD_SOFT_RTD, i_last);

                    // jacobians
                    for (int ii = 0; ii < LEN_JAC_SOFT_RTD; ii++) {
                        od_(IDX_OD_JAC_SOFT_RTD+ii, i) = od_(IDX_OD_JAC_SOFT_RTD+ii, i_last);
                    }

                    // inverse priority
                    inv_prio_rtd_(i) = inv_prio_rtd_(i_last);
                }
            }
        }
    }
    else {
        // zero
        od_.block(IDX_OD_SOFT_RTD, 0, 1, ACADO_N+1);
        od_.block(IDX_OD_JAC_SOFT_RTD, 0, LEN_JAC_SOFT_RTD, ACADO_N+1);
        inv_prio_rtd_.setOnes();
    }
} // evaluateRTDObjective

void NonlinearMPC::findNearestRayOrigins()
{
    /*  !! MUST BE CALLED EVERY TIME ONE OF THESE PARAMETERS CHANGE:
        ray_casting_interval_
    */

    // for all MPC nodes
    for (int i = 0; i < ACADO_N+1; i++) {
        // find nearest ray origin
        nearest_ray_origin_(i) = round(double(i)/ray_casting_interval_);
    }
} // findNearestRayOrigins

void NonlinearMPC::preEvaluateObjectives()
{
    updateObjectiveParameters();

    calculateSpeedStatesHorizon(); // so we dont need to recalculate these constantly in the following functions

    evaluateAOAObjective(); // wrapper for externally evaluated AoA objective

    evaluateHAGLObjective(); // wrapper for externally evaluated HAGL objective

    detectOcclusions(); // wrapper for occlusion detector (needed for RTD eval)

    evaluateRTDObjective(); // wrapper for externally evaluated RTD objective
} // preEvaluateObjectives

void NonlinearMPC::setOcclusionWindows()
{
    /*  !! MUST BE CALLED EVERY TIME ONE OF THESE PARAMETERS CHANGE:
        len_occ_window_
        len_occ_data_
    */

    int left_window_incr = (len_occ_window_ - 1) / 2; // assumes forward-looking window bias for even window lengths
    for (int i = 0; i < len_occ_data_; i++) {
        // set the start index for the window
        // window is centered at the current origin
        // assume a forward-looking window bias for left/right walls
        occ_window_start_(i) = constrain(i - left_window_incr, 0, len_occ_data_-len_occ_window_);
    }
} // findNearestRayOrigins

/*
    END OBJECTIVE PRE-EVALUATION FUNCTIONS
*/

/*
    NMPC FUNCTIONS
*/

void NonlinearMPC::applyControl()
{
    // apply first NU controls
    double u_T = acadoVariables.u[IDX_U_U_T];
    double phi_ref = acadoVariables.u[IDX_U_PHI_REF];
    double theta_ref = acadoVariables.u[IDX_U_THETA_REF];
    if (std::isnan(u_T) || std::isnan(phi_ref) || std::isnan(theta_ref)) { // XXX: should probably also check for nans in state array
        // probably solver issue
        u_T = 0.0;
        phi_ref = 0.0;
        theta_ref = 0.0;
        re_init_horizon_ = true;
        obctrl_status_ = OffboardControlStatus::BAD_SOLUTION;
    }
    else {
        // constrain to bounds XXX: adapt these if necessary once we start playing with the constraints
        u_T = constrain(u_T, lb_(IDX_U_U_T), ub_(IDX_U_U_T));
        phi_ref = constrain(phi_ref, lb_(IDX_U_PHI_REF), ub_(IDX_U_PHI_REF));
        theta_ref = constrain(theta_ref, lb_(IDX_U_THETA_REF), ub_(IDX_U_THETA_REF));
    }

    u_(IDX_U_U_T) = u_T;
    u_(IDX_U_PHI_REF) = phi_ref;
    u_(IDX_U_THETA_REF) = theta_ref;
} // applyControl

void NonlinearMPC::filterControlReference()
{
    // first order filter on reference horizon driven by currently applied control held through horizon
    u_ref_ = (u_ * Eigen::Matrix<double, 1, ACADO_N>::Ones() - u_ref_) / control_parameters_.tau_u * node_parameters_.iteration_timestep + u_ref_;
} // filterControlReference

void NonlinearMPC::filterTerrainCostJacobian()
{
    // first order filter on terrain based jacobian
    for (int i = 0; i < ACADO_N+1; i++) {
        for (int j = IDX_OD_SOFT_HAGL; j < ACADO_NOD; j++) {
            acadoVariables.od[ACADO_NOD * i + j] = (od_(j, i) - acadoVariables.od[ACADO_NOD * i + j]) / control_parameters_.tau_terr * node_parameters_.iteration_timestep + acadoVariables.od[ACADO_NOD * i + j];
        }
    }
} // filterTerrainCostJacobian

int NonlinearMPC::nmpcIteration()
{
    // < -- START iteration timer
    ros::Time t_iter_start = ros::Time::now();

    obctrl_status_ = OffboardControlStatus::NOMINAL;
    ros::Duration t_elapsed;

    // check if ghost operation is enabled, else constantly re-initialize horizon //XXX: do we really even need this? could just always ghost run the mpc..
    if (!node_parameters_.ghost_en) re_init_horizon_ = true;

    // < -- START objective pre-evaluation timer
    ros::Time t_preeval_start = ros::Time::now();

    // updates
    updateAcadoX0(); // new measurements
    updateAcadoW(); // new weights (not including prioritization)
    updateAcadoConstraints(); // new constraints

    // check if something went wrong..
    int ret_solver = 0; // XXX: need to add timeout here with check on how long we are constantly trying to re-initialize (if this happens)
    if (re_init_horizon_) {
        initializeHorizon();
        ROS_ERROR("initializeHorizon: re-initializing, possibly in-air, propagate current state measurements forward");
        re_init_horizon_ = false;
    }
    else {
        // only filter the control reference when we are not re-initializing (i.e. there exists a "previous" control ref to filter)
        if (control_parameters_.use_floating_ctrl_ref) filterControlReference(); // from previously applied control
    }

    // pre-evaluate select objectives (external to ACADO model)
    preEvaluateObjectives();

    // set any non-zero objective references (e.g. call guidance logic)
    setObjectiveReferences();

    // objective prioritization
    prioritizeObjectives();

    // update objective references
    updateAcadoY();

    // update online data (DEPENDS on preEvaluateObjectives)
    updateAcadoOD();

    t_elapsed = ros::Time::now() - t_preeval_start;
    uint64_t t_preeval = t_elapsed.toNSec()/1000;
    // END objective pre-evaluation timer -- >

    /* ACADO solver functions */

    // SOLVER: prepare ACADO workspace for next iteration step
    // < -- START preparation step timer
    ros::Time t_prep_start = ros::Time::now();
    err_code_preparation_step_ = acado_preparationStep();
    if ( err_code_preparation_step_ != 0 ) {
        ROS_ERROR("nmpc iteration: preparation step error, code %d", err_code_preparation_step_);
        obctrl_status_ = OffboardControlStatus::PREPARATION_STEP_ERROR;
        re_init_horizon_ = true;
    }
    t_elapsed = ros::Time::now() - t_prep_start;
    uint64_t t_prep = t_elapsed.toNSec()/1000;
    // END preparation step timer -- >

    // SOLVER: perform the feedback step
    // < -- START feedback step timer
    ros::Time t_fb_start = ros::Time::now();
    err_code_feedback_step_ = acado_feedbackStep();
    if ( err_code_feedback_step_ != 0 ) {
        ROS_ERROR("nmpc iteration: feedback step error, code %d", err_code_feedback_step_);
        obctrl_status_ = OffboardControlStatus::FEEDBACK_STEP_ERROR;
        re_init_horizon_ = true;
    }
    t_elapsed = ros::Time::now() - t_fb_start;
    uint64_t t_fb = t_elapsed.toNSec()/1000;
    // END feedback step timer -- >

    /* publishing */

    // send controls to vehicle
    applyControl();
    if (!re_init_horizon_) publishControls(u_(IDX_U_U_T), u_(IDX_U_PHI_REF), u_(IDX_U_THETA_REF)); // don't publish if a re_init is necessary (XXX: might put a timeout here at some point..)
    t_elapsed = ros::Time::now() - t_last_ctrl_;
    uint64_t t_ctrl = t_elapsed.toNSec()/1000;
    t_last_ctrl_ = ros::Time::now(); // record directly after publishing control

    // store current nmpc states
    publishNMPCStates();

    // record timing / nmpc info
    publishNMPCInfo(t_iter_start, t_ctrl, t_preeval, t_prep, t_fb);

    // publish visualization msgs
    if (node_parameters_.viz_en) publishNMPCVisualizations();

    return ret_solver;
} // nmpcIteration

void NonlinearMPC::prioritizeObjectives()
{
    /*
        prioritizes select objectives by adapting specific weights (external to the model jacobian)
    */

    // if still on ground (landed), do not consider terrain
    if ((landed_state_ == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
        || (landed_state_ == mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED)
        || !(control_parameters_.enable_terrain_feedback && grid_map_valid_)) { //TODO: check when, if ever, the undefined state is set by pixhawk
        inv_prio_hagl_.setOnes();
        inv_prio_rtd_.setOnes();

        // terrain objective weighting through horizon (row-major array)
        for (int i=0; i<ACADO_N; i++) {
            acadoVariables.W[ACADO_NY*ACADO_NY*i+ACADO_NY*IDX_Y_SOFT_HAGL+IDX_Y_SOFT_HAGL] = 0.0;
            acadoVariables.W[ACADO_NY*ACADO_NY*i+ACADO_NY*IDX_Y_SOFT_RTD+IDX_Y_SOFT_RTD] = 0.0;
        }
    }

    // unit velocity objective weighting through horizon (row-major array)
    bool manual_roll = manual_control_sp_.enabled && manual_control_sp_.lat_input == ManualControlLatInputs::ROLL;
    if (manual_roll) {
        // de-prioritize path following objectives when near terrain
        // zero out lateral-directional unit ground velocity weights as they are not used when roll is commanded
        for (int i=0; i<ACADO_N; i++) {
            acadoVariables.W[ACADO_NY*ACADO_NY*i+ACADO_NY*IDX_Y_VN+IDX_Y_VN] *= 0.0;
            acadoVariables.W[ACADO_NY*ACADO_NY*i+ACADO_NY*IDX_Y_VE+IDX_Y_VE] *= 0.0;
            acadoVariables.W[ACADO_NY*ACADO_NY*i+ACADO_NY*IDX_Y_VD+IDX_Y_VD] *= inv_prio_hagl_(i) * inv_prio_rtd_(i);
        }
        acadoVariables.WN[ACADO_NYN*IDX_Y_VN+IDX_Y_VN] *= 0.0;
        acadoVariables.WN[ACADO_NYN*IDX_Y_VE+IDX_Y_VE] *= 0.0;
        acadoVariables.WN[ACADO_NYN*IDX_Y_VD+IDX_Y_VD] *= inv_prio_hagl_(ACADO_N) * inv_prio_rtd_(ACADO_N);
    }
    else {
        // de-prioritize path following objectives when near terrain
        double inv_prio;
        for (int i=0; i<ACADO_N; i++) {
            inv_prio = inv_prio_hagl_(i) * inv_prio_rtd_(i);
            acadoVariables.W[ACADO_NY*ACADO_NY*i+ACADO_NY*IDX_Y_VN+IDX_Y_VN] *= inv_prio;
            acadoVariables.W[ACADO_NY*ACADO_NY*i+ACADO_NY*IDX_Y_VE+IDX_Y_VE] *= inv_prio;
            acadoVariables.W[ACADO_NY*ACADO_NY*i+ACADO_NY*IDX_Y_VD+IDX_Y_VD] *= inv_prio;
        }
        inv_prio = inv_prio_hagl_(ACADO_N) * inv_prio_rtd_(ACADO_N);
        acadoVariables.WN[ACADO_NYN*IDX_Y_VN+IDX_Y_VN] *= inv_prio;
        acadoVariables.WN[ACADO_NYN*IDX_Y_VE+IDX_Y_VE] *= inv_prio;
        acadoVariables.WN[ACADO_NYN*IDX_Y_VD+IDX_Y_VD] *= inv_prio;
    }

    // de-prioritize feed-forward roll reference //XXX: is there a way to do this back where it is set? (currently not.. as prio is called after eval)
    bool ff_roll = !control_parameters_.use_floating_ctrl_ref && control_parameters_.use_ff_roll_ref && path_sp_.type == PathTypes::LOITER;
    if (manual_roll || ff_roll) {
        for (int i=0; i<ACADO_N; i++) {
            u_ref_(IDX_U_PHI_REF, i) *= inv_prio_hagl_(i) * inv_prio_rtd_(i);
        }
    }
} // prioritizeObjectives

void NonlinearMPC::setObjectiveReferences()
{
    /*
        DEPENDS ON:
        evaluateHAGLObjective()
        detectOcclusions()
        evaluateRTDObjective()
    */

    // allocate
    Eigen::Vector3d unit_ground_vel_sp(1.0, 0.0, 0.0);
    Eigen::Vector3d ground_vel(0.0, 0.0, 0.0);
    double err_lat = 0.0;
    double err_lon = 0.0;
    double unit_err_lat = 0.0;

    // for all nodes
    for (int i = 0; i < ACADO_N+1; i++) {
        /* velocity reference */

        // vehicle position
        Eigen::Vector3d veh_pos(acadoVariables.x[i * ACADO_NX + IDX_X_POS], acadoVariables.x[i * ACADO_NX + IDX_X_POS+1],
            acadoVariables.x[i * ACADO_NX + IDX_X_POS+2]);

        // ground velocity
        ground_vel(0) = spds_[i].ground_vel[0];
        ground_vel(1) = spds_[i].ground_vel[1];
        ground_vel(2) = spds_[i].ground_vel[2];

        // calculate the unit velocity reference at the current node
        gl_.calculateUnitVelocityReference(unit_ground_vel_sp, err_lat, err_lon, unit_err_lat,
            veh_pos, ground_vel, spds_[i].ground_sp_lat, terrain_alt_horizon_(i), manual_control_sp_, path_sp_);

        if (gl_.getUseOCCAsGuidance()) {
            // steer current guidance vector with terrain cost jacobians
            gl_.augmentTerrainCostToGuidance(unit_ground_vel_sp, od_.block(IDX_OD_SOFT_RTD, i, 3, 1).normalized(), inv_prio_rtd_(i));
        }

        // set the unit ground velocity reference
        if (i < ACADO_N) {
            y_(IDX_Y_VN, i) = unit_ground_vel_sp(0);
            y_(IDX_Y_VE, i) = unit_ground_vel_sp(1);
            y_(IDX_Y_VD, i) = unit_ground_vel_sp(2);
        }
        else {
            yN_(IDX_Y_VN) = unit_ground_vel_sp(0);
            yN_(IDX_Y_VE) = unit_ground_vel_sp(1);
            yN_(IDX_Y_VD) = unit_ground_vel_sp(2);
        }

        /* control reference */
        if (!control_parameters_.use_floating_ctrl_ref && i<ACADO_N) {
            // fixed control reference

            // pitch reference
            u_ref_(IDX_U_THETA_REF, i) = control_parameters_.fixed_pitch_ref; //TODO: in future could schedule these also with airsp / fpa / roll*** setpoints

            // throttle reference
            u_ref_(IDX_U_U_T, i) = control_parameters_.fixed_throt_ref; //TODO: in future could schedule these also with airsp / fpa setpoints

            // roll reference
            if (manual_control_sp_.enabled && manual_control_sp_.lat_input == ManualControlLatInputs::ROLL) {
                // we are using roll reference to steer the aircraft in manual mode
                u_ref_(IDX_U_PHI_REF, i) = manual_control_sp_.roll;
            }
            else if (control_parameters_.use_ff_roll_ref && path_sp_.type == PathTypes::LOITER && !manual_control_sp_.enabled) {
                // feed-forward the path curvature when near to the track
                const double lat_accel_ff = spds_[i].ground_sp_lat_sq / path_sp_.signed_radius / ONE_G;
                const double track_proximity = 0.5 * (1.0 + cos(M_PI * unit_err_lat));
                u_ref_(IDX_U_PHI_REF, i) = atan(lat_accel_ff) * track_proximity;
            }
            else {
                u_ref_(IDX_U_PHI_REF, i) = 0.0;
            }
        }
    } // end i loop
} // setObjectiveReferences

void NonlinearMPC::updateAcadoConstraints()
{
    // update hard constraints

    // TODO: adapt constraints as necessary, e.g. for landing

    for (int i=0; i<ACADO_N; i++) {
        acadoVariables.lbValues[ACADO_NU * i + IDX_U_U_T] = lb_(IDX_U_U_T);
        acadoVariables.lbValues[ACADO_NU * i + IDX_U_PHI_REF] = lb_(IDX_U_PHI_REF);
        acadoVariables.lbValues[ACADO_NU * i + IDX_U_THETA_REF] = lb_(IDX_U_THETA_REF);

        acadoVariables.ubValues[ACADO_NU * i + IDX_U_U_T] = ub_(IDX_U_U_T);
        acadoVariables.ubValues[ACADO_NU * i + IDX_U_PHI_REF] = ub_(IDX_U_PHI_REF);
        acadoVariables.ubValues[ACADO_NU * i + IDX_U_THETA_REF] = ub_(IDX_U_THETA_REF);
    }

} // updateAcadoConstraints

void NonlinearMPC::updateAcadoOD()
{
    /*
        update ACADO online data

        NOTE: pre-evaluated objectives/jacobians set in preEvaluateObjectives() -- this should be run BEFORE this update function
    */

    // air density
    od_.block(IDX_OD_RHO, 0, 1, ACADO_N+1).setConstant(calcAirDensity(static_pressure_pa_, temperature_c_));

    // wind
    od_.block(IDX_OD_W, 0, 3, ACADO_N+1) = x0_wind_ * Eigen::Matrix<double, 1, ACADO_N+1>::Ones();

    // control-augmented model dynamics
    od_.block(IDX_OD_TAU_PHI, 0, 1, ACADO_N+1).setConstant(model_parameters_.tau_roll);
    od_.block(IDX_OD_TAU_THETA, 0, 1, ACADO_N+1).setConstant(model_parameters_.tau_pitch);
    od_.block(IDX_OD_K_PHI, 0, 1, ACADO_N+1).setConstant(model_parameters_.k_roll);
    od_.block(IDX_OD_K_THETA, 0, 1, ACADO_N+1).setConstant(model_parameters_.k_pitch);

    // prop delay
    od_.block(IDX_OD_TAU_N, 0, 1, ACADO_N+1).setConstant(model_parameters_.tau_n_prop);

    // symmetric flaps setting
    od_.block(IDX_OD_DELTA_F, 0, 1, ACADO_N+1).setConstant(flapsToRad(flaps_normalized_));

    // time delayed terrain cost jacobians
    filterTerrainCostJacobian();

    Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N+1>>(const_cast<double*>(acadoVariables.od)) = od_;
} // updateAcadoOD

void NonlinearMPC::updateAcadoW()
{
    // NOTE: this does not yet include any weight priorities

    // objective weighting through horizon (row-major array)
    for (int i=0; i<ACADO_N; i++) {
        for (int j=0; j<ACADO_NY; j++) {
            acadoVariables.W[ACADO_NY*ACADO_NY*i+ACADO_NY*j+j] = w_(j) * inv_y_scale_sq_(j);
        }
    }

    // end term objective weighting through horizon (row-major array)
    for (int j=0; j<ACADO_NY; j++) {
        acadoVariables.WN[ACADO_NYN*j+j] = w_(j) * inv_y_scale_sq_(j);
    }

} // updateAcadoW

void NonlinearMPC::updateAcadoX0()
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
    if (airsp < veh_parameters_.airsp_thres) {
        // airspeed is too low - bound it to minimum value
        x0_(IDX_X_V) = veh_parameters_.airsp_thres;
        if (airsp < 0.1) { // XXX: magic number
            // if reeeally zero - define airspeed vector in yaw direction with zero flight path angle
            x0_(IDX_X_GAMMA) = 0.0; // flight path angle
            x0_(IDX_X_XI) = unwrapHeading(x0_euler_(2), re_init_horizon_); // heading
        }
        else {
            x0_(IDX_X_GAMMA) = -asin(airsp_vec(2)/airsp); // flight path angle
            x0_(IDX_X_XI) = unwrapHeading(atan2(airsp_vec(1),airsp_vec(0)), re_init_horizon_); // heading
        }
    }
    else {
        x0_(IDX_X_V) = airsp; // airspeed
        x0_(IDX_X_GAMMA) = -asin(airsp_vec(2)/airsp); // flight path angle
        x0_(IDX_X_XI) = unwrapHeading(atan2(airsp_vec(1),airsp_vec(0)), re_init_horizon_); // heading
    }

    // attitude
    x0_(IDX_X_PHI) = x0_euler_(0); // roll
    x0_(IDX_X_THETA) = x0_euler_(1); // pitch

    // prop speed
    if (re_init_horizon_) {
        // take current state
        const double throt = mapPX4ThrotToNormalizedThrot(px4_throt_, airsp, x0_(IDX_X_THETA) - x0_(IDX_X_GAMMA));
        n_prop_virt_ = mapNormalizedThrotToPropSpeed(throt, airsp, x0_(IDX_X_THETA) - x0_(IDX_X_GAMMA));
    }
    else if (offboard_mode_) {
        // the MPC is in control, propagate from last state
        propagateVirtPropSpeed(u_(IDX_U_U_T), acadoVariables.x[IDX_X_V], acadoVariables.x[IDX_X_THETA] - acadoVariables.x[IDX_X_GAMMA]);
    }
    else {
        // PX4 is commanding the vehicle
        propagateVirtPropSpeed(mapPX4ThrotToNormalizedThrot(px4_throt_, airsp, x0_(IDX_X_THETA) - x0_(IDX_X_GAMMA)), acadoVariables.x[IDX_X_V], acadoVariables.x[IDX_X_THETA] - acadoVariables.x[IDX_X_GAMMA]);
    }
    x0_(IDX_X_NPROP) = n_prop_virt_;

    Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>>(const_cast<double*>(acadoVariables.x0)) = x0_;
} // updateAcadoX0

void NonlinearMPC::updateAcadoY()
{
    // NOTE: run AFTER pre-evaluation of objectives

    // airspeed reference
    y_.block(IDX_Y_V, 0, 1, ACADO_N).setConstant(control_parameters_.airsp_ref);
    yN_(IDX_Y_V) = (manual_control_sp_.enabled) ? manual_control_sp_.airspeed : control_parameters_.airsp_ref;

    // attitude reference
    y_.block(IDX_Y_PHI, 0, 1, ACADO_N) = u_ref_.block(IDX_U_PHI_REF, 0, 1, ACADO_N);
    y_.block(IDX_Y_THETA, 0, 1, ACADO_N) = u_ref_.block(IDX_U_THETA_REF, 0, 1, ACADO_N);
    yN_(IDX_Y_PHI) = u_ref_(IDX_U_PHI_REF, ACADO_N);
    yN_(IDX_Y_THETA) = u_ref_(IDX_U_THETA_REF, ACADO_N);

    // control objective references
    y_.block(IDX_Y_U_T, 0, 1, ACADO_N) = u_ref_.block(IDX_U_U_T, 0, 1, ACADO_N);
    y_.block(IDX_Y_PHI_REF, 0, 1, ACADO_N) = u_ref_.block(IDX_U_PHI_REF, 0, 1, ACADO_N);
    y_.block(IDX_Y_THETA_REF, 0, 1, ACADO_N) = u_ref_.block(IDX_U_THETA_REF, 0, 1, ACADO_N);

    // NOTE: unit ground velocity references set in objective pre-evaluation

    Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) = y_;
    Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) = yN_;
} // updateAcadoY

void NonlinearMPC::updateObjectiveParameters()
{
    // NOTE: depends on the current (un-modulated) weighting - evaluate PRIOR to updating current weighting priorities

    // soft AoA params
    huber_aoa_p_.updateScale(acadoVariables.W[ACADO_NY * IDX_Y_SOFT_AOA + IDX_Y_SOFT_AOA]);
    huber_aoa_m_.updateScale(acadoVariables.W[ACADO_NY * IDX_Y_SOFT_AOA + IDX_Y_SOFT_AOA]);

    // soft HAGL params
    huber_hagl_.updateScale(acadoVariables.W[ACADO_NY * IDX_Y_SOFT_HAGL + IDX_Y_SOFT_HAGL]);

    // soft RTD params
    const double inv_g_tan_roll = 1.0 / tan(constrain(control_parameters_.roll_lim_rad, 0.01, M_PI_2)) / ONE_G;
    huber_rtd_params_.constr_scaler = huber_rtd_params_.constr_gain * inv_g_tan_roll;
    huber_rtd_params_.delta_scaler = huber_rtd_params_.delta_gain * inv_g_tan_roll;

    /* path reference */ // TODO: should probably be set with some kind of service instead of polling every iteration
    int temp_int;
    nmpc_.getParam("/nmpc/path/path_type", temp_int);
    path_sp_.type = PathTypes(temp_int);
    nmpc_.getParam("/nmpc/path/b_n", path_sp_.pos(0));
    nmpc_.getParam("/nmpc/path/b_e", path_sp_.pos(1));
    nmpc_.getParam("/nmpc/path/b_d", path_sp_.pos(2));
    nmpc_.getParam("/nmpc/path/Gamma_p", path_sp_.fpa);
    path_sp_.fpa *= DEG_TO_RAD;
    double temp_double;
    nmpc_.getParam("/nmpc/path/chi_p", temp_double);
    path_sp_.bearing = temp_double * DEG_TO_RAD;
    path_sp_.signed_radius = temp_double; // XXX: the static param is hijacked in the case of loiter vs line atm
} // updateObjectiveParameters

/*
    END NMPC FUNCTIONS
*/

/*
    INITIALIZATION
*/

void NonlinearMPC::initializeVariables()
{
    x0_pos_.setZero();
    x0_vel_ << 15.0, 0.0, 0.0;
    x0_euler_.setZero();
    x0_wind_.setZero();

    for (int i = 0; i < MAX_SIZE_TERR_ARRAY; i++) map_array_[i] = 0.0;

    x0_.setZero();
    u_.setZero();
    u_ref_.setZero();
    y_.setZero();
    yN_.setZero();
    od_.setZero();

    updateAcadoX0();
    updateAcadoConstraints();
    updateAcadoW();
    updateAcadoOD();
    updateAcadoY();
} // initializeVariables

void NonlinearMPC::initializeHorizon()
{
    for (int i = 0; i < ACADO_N; i++) {

        // propagate position linearly at current ground velocity
        for (int j = IDX_X_POS; j < IDX_X_POS+3; ++j) {
            acadoVariables.x[ACADO_NX * i + j] = x0_pos_(j) + x0_vel_(j) * node_parameters_.nmpc_discretization * i;
        }

        // hold velocity axis constant through horizon
        acadoVariables.x[ACADO_NX * i + IDX_X_V] = x0_(IDX_X_V);
        acadoVariables.x[ACADO_NX * i + IDX_X_GAMMA] = x0_(IDX_X_GAMMA);
        acadoVariables.x[ACADO_NX * i + IDX_X_XI] = x0_(IDX_X_XI);

        // hold attitude constant (within bounds) through horizon
        double roll = constrain(x0_(IDX_X_PHI), lb_(IDX_U_PHI_REF), ub_(IDX_U_PHI_REF));
        double pitch = constrain(x0_(IDX_X_THETA), lb_(IDX_U_THETA_REF), ub_(IDX_U_THETA_REF));
        acadoVariables.x[ACADO_NX * i + IDX_X_PHI] = roll;
        acadoVariables.x[ACADO_NX * i + IDX_X_THETA] = pitch;

        // hold current throttle constant through horizon
        double throt = mapPX4ThrotToNormalizedThrot(px4_throt_, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
        double n_prop = mapNormalizedThrotToPropSpeed(throt, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
        acadoVariables.x[ACADO_NX * i + IDX_X_NPROP] = n_prop;

        // controls
        acadoVariables.u[ACADO_NU * i + IDX_U_U_T] = throt;
        acadoVariables.u[ACADO_NU * i + IDX_U_PHI_REF] = 0.0;
        acadoVariables.u[ACADO_NU * i + IDX_U_THETA_REF] = 0.0;
    }

    // propagate position linearly at current ground velocity
    for (int j = IDX_X_POS; j < IDX_X_POS+3; ++j) {
        acadoVariables.x[ACADO_NX * ACADO_N + j] = x0_pos_(j) + x0_vel_(j) * node_parameters_.nmpc_discretization * ACADO_N;
    }

    // hold velocity axis constant through horizon
    acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_V] = x0_(IDX_X_V);
    acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_GAMMA] = x0_(IDX_X_GAMMA);
    acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_XI] = x0_(IDX_X_XI);

    // hold attitude constant (within bounds) through horizon
    double roll = constrain(x0_(IDX_X_PHI), lb_(IDX_U_PHI_REF), ub_(IDX_U_PHI_REF));
    double pitch = constrain(x0_(IDX_X_THETA), lb_(IDX_U_THETA_REF), ub_(IDX_U_THETA_REF));
    acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_PHI] = roll;
    acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_THETA] = pitch;

    // hold current throttle constant through horizon
    double throt = mapPX4ThrotToNormalizedThrot(px4_throt_, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
    double n_prop = mapNormalizedThrotToPropSpeed(throt, x0_(IDX_X_V), pitch - x0_(IDX_X_GAMMA));
    acadoVariables.x[ACADO_NX * ACADO_N + IDX_X_NPROP] = n_prop;
} // initializeHorizon

int NonlinearMPC::initializeNMPC()
{
    int ret = 0;

    /* initialize the solver */
    ret = acado_initializeSolver();
    if (ret) {
        ROS_ERROR("initNMPC: failed to initialize acado solver");
        return ret;
    }

    /* initialize fixed parameters */
    ret = initializeParameters();
    if (ret) {
        ROS_ERROR("initializeNMPC: failed to initialize parameters");
        return ret;
    }
    else {
        ROS_ERROR("initializeNMPC: parameters initialized");
    }

    /* initialize ACADO variables */
    initializeVariables();
    ROS_ERROR("initializeNMPC: variables initialized");

    /* initialize the horizon */
    initializeHorizon();
    ROS_ERROR("initializeNMPC: horizon initialized");

    return ret;
} // initializeNMPC

int NonlinearMPC::initializeParameters()
{
    /* load parameters from server */

    double temp_double;
    int temp_int;

    // node parameters

    if (!nmpc_.getParam("/nmpc/loop_rate", node_parameters_.nmpc_iteration_rate)) {
        ROS_ERROR("initParameters: nmpc iteration rate not found");
        return 1;
    }
    if (node_parameters_.nmpc_iteration_rate < 1.0 || node_parameters_.nmpc_iteration_rate > 100.0) {
        // sanity check
        ROS_ERROR("initParameters: nmpc iteration rate exceeds bounds");
        node_parameters_.nmpc_iteration_rate = constrain(node_parameters_.nmpc_iteration_rate, 1.0, 100.0); // Hz
    }
    node_parameters_.iteration_timestep = 1.0 / node_parameters_.nmpc_iteration_rate; // s
    if (!nmpc_.getParam("/nmpc/time_step", node_parameters_.nmpc_discretization)) {
        ROS_ERROR("initParameters: nmpc discretization time step not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/viz/enable", temp_int)) {
        ROS_ERROR("initParameters: visualization setting not found");
        return 1;
    }
    node_parameters_.viz_en = temp_int;
    if (!nmpc_.getParam("/nmpc/enable_ghost", temp_int)) {
        ROS_ERROR("initParameters: ghost setting not found");
        return 1;
    }
    node_parameters_.ghost_en = temp_int;

    // vehicle parameters

    if (!nmpc_.getParam("/nmpc/veh/airsp_thres", veh_parameters_.airsp_thres)) {
        ROS_ERROR("initParameters: airspeed threshold not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/veh/airsp_min", veh_parameters_.airsp_min)) {
        ROS_ERROR("initParameters: airspeed minimum not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/veh/airsp_max", veh_parameters_.airsp_max)) {
        ROS_ERROR("initParameters: airspeed maximum not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/veh/flaps_lim", temp_double)) {
        ROS_ERROR("initParameters: flaps limit not found");
        return 1;
    }
    veh_parameters_.flaps_lim_rad = temp_double * DEG_TO_RAD;

    // propeller parameters

    if (!nmpc_.getParam("/nmpc/prop/n_0", prop_parameters_.n_0)) {
        ROS_ERROR("initParameters: n_0 not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/prop/n_max", prop_parameters_.n_max)) {
        ROS_ERROR("initParameters: n_max not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/prop/n_delta", prop_parameters_.n_delta)) {
        ROS_ERROR("initParameters: n_delta not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/prop/vpmin", prop_parameters_.vp_min)) {
        ROS_ERROR("initParameters: vp_min not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/prop/vpmax", prop_parameters_.vp_max)) {
        ROS_ERROR("initParameters: vp_max not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/prop/n_T0_vmin", prop_parameters_.n_T0_vmin)) {
        ROS_ERROR("initParameters: n_T0_vmin not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/prop/n_T0_vmax", prop_parameters_.n_T0_vmax)) {
        ROS_ERROR("initParameters: n_T0_vmax not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/prop/epsilon_T", temp_double)) {
        ROS_ERROR("initParameters: epsilon_T not found");
        return 1;
    }
    prop_parameters_.epsilon_T_rad = temp_double * DEG_TO_RAD;

    // control-augmented model dynamics

    if (!nmpc_.getParam("/nmpc/od/tau_phi", model_parameters_.tau_roll)) {
        ROS_ERROR("initParameters: tau_phi not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/od/tau_theta", model_parameters_.tau_pitch)) {
        ROS_ERROR("initParameters: tau_theta found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/od/k_phi", model_parameters_.k_roll)) {
        ROS_ERROR("initParameters: k_phi not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/od/k_theta", model_parameters_.k_pitch)) {
        ROS_ERROR("initParameters: k_theta not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/od/tau_n", model_parameters_.tau_n_prop)) {
        ROS_ERROR("initParameters: tau_n not found");
        return 1;
    }

    // inverse of state squared output scales

    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/vn", inv_y_scale_sq_(IDX_Y_VN))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/vn not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/vd", inv_y_scale_sq_(IDX_Y_VD))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/vd not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/ve", inv_y_scale_sq_(IDX_Y_VE))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/ve not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/v", inv_y_scale_sq_(IDX_Y_V))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/v not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/phi", inv_y_scale_sq_(IDX_Y_PHI))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/phi not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/theta", inv_y_scale_sq_(IDX_Y_THETA))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/theta not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/soft_aoa", inv_y_scale_sq_(IDX_Y_SOFT_AOA))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/soft_aoa not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/soft_h", inv_y_scale_sq_(IDX_Y_SOFT_HAGL))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/soft_h not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/soft_r", inv_y_scale_sq_(IDX_Y_SOFT_RTD))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/soft_r not found");
        return 1;
    }

    // control squared output scales

    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/u_T", inv_y_scale_sq_(IDX_Y_U_T))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/u_T not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/phi_ref", inv_y_scale_sq_(IDX_Y_PHI_REF))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/phi_ref not found");
        return 1;
    }
    if (!nmpc_.getParam("/nmpc/inv_y_scale_sq/theta_ref", inv_y_scale_sq_(IDX_Y_THETA_REF))) {
        ROS_ERROR("initParameters: inv_y_scale_sq/theta_ref not found");
        return 1;
    }

    // hard constraints
    if (!nmpc_.getParam("/nmpc/lb/u_T", temp_double)) {
        ROS_ERROR("initParameters: lb/u_T not found");
        return 1;
    }
    lb_(IDX_U_U_T) = constrain(temp_double, 0.0, 1.0);
    if (!nmpc_.getParam("/nmpc/ub/u_T", temp_double)) {
        ROS_ERROR("initParameters: ub/u_T not found");
        return 1;
    }
    ub_(IDX_U_U_T) = constrain(temp_double, lb_(IDX_U_U_T), 1.0);

    return 0;
} // initializeParameters

/*
    END INITIALIZATION
*/

/*
    NODE
*/

void NonlinearMPC::shutdown() {
    ROS_ERROR("Shutting down NMPC...");
    ros::shutdown();
} // shutdown

/*
    END NODE
*/

} // namespace fw_nmpc

int main(int argc, char **argv)
{

    /* initialize node */
    ros::init(argc, argv, "fw_nmpc");
    fw_nmpc::NonlinearMPC nmpc;

    ros::spinOnce();

    /* wait for required subscriptions */
    nmpc.checkSubs();

    /* initialize states, params, and solver */
    int ret = nmpc.initializeNMPC();
    if (ret != 0) {
        ROS_ERROR("initializeNMPC: failed to prime solver.");
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
            ROS_ERROR("nmpc iteration: failed to prime solver.");
            return 1;
        }

        /* sleep */
        nmpc_rate.sleep();
    }

    ROS_ERROR("fw nmpc: closing...");
    return 0;
} // main
