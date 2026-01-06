
  /**
   * @file simple_drone_plugin.cpp
   * @brief Gazebo plugin for simplified 2D quadrotor control under wind disturbances
   *
   * OVERVIEW:
   *   Implements a Gazebo ModelPlugin for Crazyflie 2.1 dynamics with planar XY
   *   motion and altitude-locked Z-axis. Receives acceleration commands from ROS2
   *   controllers and applies forces to the Gazebo physics engine. Publishes full
   *   state odometry for feedback control loops.
   *
   * PHYSICS MODEL:
   *   XY-plane:  F = m * a_cmd + F_wind + F_drag
   *   Z-axis:    F_z = m*g + K_p*(z_ref - z) + K_d*(-dz/dt)  [altitude hold]
   *   Reference altitude: z_ref = 0.5 m (configurable via SDF)
   *   Altitude gains: K_p = 12.0, K_d = 4.0 (aggressive for fast Z response)
   *
   * ADVANCED FEATURES:
   *   - Actuator lag: First-order filter with asymmetric tau (tau_up, tau_down)
   *   - Drag model: Blended linear + quadratic (speed-dependent)
   *   - Wind injection: Per-drone wind topics for spatial coherence
   *   - Sensor noise: Gaussian noise on position/velocity (Type-2 Fuzzy testing)
   *   - Motor model: Optional sim_cf2 rotor dynamics (use_motor_model flag)
   *
   * ROS2 INTERFACE:
   *   Subscribe: /{namespace}/cmd_accel       (geometry_msgs/Vector3, ~200 Hz)
   *              /{namespace}/wind/velocity    (geometry_msgs/Vector3, ~50 Hz)
   *              /{namespace}/wind/force       (geometry_msgs/Vector3, ~50 Hz)
   *              /wind/velocity                (global fallback)
   *              /wind/force                   (global fallback)
   *   Publish:   /{namespace}/odom             (nav_msgs/Odometry, ~1000 Hz)
   *
   * TESTED SCENARIOS (6-Phase Framework):
   *   Phase 1: BASELINE      - No wind (reference performance)
   *   Phase 2: STEADY_WIND   - Constant 3 m/s @ 45° (DC rejection)
   *   Phase 3: TURBULENCE    - Von Karman turbulence, TI ∈ {0.15, 0.25, 0.35}
   *   Phase 4: GUST          - Periodic gusts (5 m/s, 1.5s duration, 10s interval)
   *   Phase 5: COMBINED      - Stochastic: turbulence + gusts + direction wander
   *   Phase 6: ENDURANCE     - Phase 5 extended to 300 seconds
   *
   * ASSUMPTIONS:
   *   - Small angle approximation (roll, pitch < 15 deg)
   *   - No rotor dynamics modeled (unless use_motor_model=true)
   *   - Wind acts as external force disturbance (no torque)
   *   - Inertia matrix is diagonal (decoupled axes)
   *   - Collision avoidance handled externally via ROS2 topic remapping (not in plugin)
   *
   * THREAD SAFETY:
   *   cmd_accel_ and wind_velocity_ are accessed from Gazebo's physics thread
   *   and ROS2 callback threads. Protected by implicit message copying.
   *
   * SDF PARAMETERS (configurable from world file):
   *   - namespace:              ROS2 namespace (e.g., "agent_0")
   *   - target_altitude:        Altitude hold setpoint [m] (default: 0.5)
   *   - actuator_tau:           Actuator lag time constant [s] (default: 0.01)
   *   - actuator_tau_up/down:   Asymmetric lag [s] (default: 0.006, 0.015)
   *   - drag_coeff_lin:         Linear drag coefficient [N/(m/s)] (default: 0.005)
   *   - drag_coeff_quad:        Quadratic drag coefficient [N/(m/s)²] (default: 0.01)
   *   - position_noise_std:     Position noise std dev [m] (default: 0.0)
   *   - velocity_noise_std:     Velocity noise std dev [m/s] (default: 0.0)
   *   - use_motor_model:        Enable sim_cf2 motor dynamics (default: false)
   *
   * EXTERNAL COMPONENTS (not part of this plugin):
   *   - Collision avoidance: collision_safety_layer_node (APF-based, ROS2 level)
   *   - Formation control: formation_coordinator_node (waypoint generation)
   *   - Wind generation: wind_publisher.py (phase-specific profiles)
   *   - Metrics logging: phase_metrics_logger.py (performance tracking)
   *
   * REFERENCES:
   *   [1] Crazyflie 2.1 specifications: https://www.bitcraze.io/
   *   [2] Gazebo ModelPlugin API: http://gazebosim.org/tutorials?tut=plugins_model
   *   [3] MIL-F-8785C: Military Specification - Flying Qualities of Piloted Airplanes
   *
   * @author Atakan Yaman
   * @date 2025-01-05
   * @version 2.0 (ETC-enabled, 6-phase framework, motor model support)
   */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <random>
#include <memory>
#include <array>

#include "agent_control_pkg/core/physics_types.hpp"
#include "agent_control_pkg/core/drone_physics_core.hpp"
#include "agent_control_pkg/core/motor_model.hpp"
#include "agent_control_pkg/core/motor_mixer.hpp"

namespace gazebo
{
  /**
   * @class SimpleDronePlugin
   * @brief Gazebo ModelPlugin
   *
   * This plugin provides the physics layer for formation control testing.
   * It converts ROS2 acceleration commands into Gazebo forces and publishes
   * full state feedback via odometry messages.
   */
  class SimpleDronePlugin : public ModelPlugin
  {
  public:
    SimpleDronePlugin() : ModelPlugin() {}

    /**
     * @brief Load plugin and initialize ROS2 interface
     * @param _model Pointer to the Gazebo model (drone)
     * @param _sdf SDF element containing plugin configuration
     *
     * This function is called once when the plugin is instantiated.
     * It sets up the ROS2 subscribers/publishers and connects to
     * Gazebo's update loop.
     */
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      // Store model pointer and get base_link (main body of drone)
      this->model_ = _model;

      // Try direct link first, then nested model link (for <include> usage)
      this->link_ = this->model_->GetLink("base_link");
      if (!this->link_)
      {
        // When using <include>, link names get prefixed with nested model name
        // Try common nested patterns: crazyflie::base_link, etc.
        auto links = this->model_->GetLinks();
        for (const auto& link : links)
        {
          std::string linkName = link->GetName();
          if (linkName.find("base_link") != std::string::npos)
          {
            this->link_ = link;
            break;
          }
        }
      }

      if (!this->link_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("SimpleDronePlugin"),
                     "base_link not found in model! Available links:");
        auto links = this->model_->GetLinks();
        for (const auto& link : links)
        {
          RCLCPP_ERROR(rclcpp::get_logger("SimpleDronePlugin"),
                       "  - %s", link->GetName().c_str());
        }
        return;
      }

      // Get drone mass from inertial properties (used for F=ma)
      this->mass_ = this->link_->GetInertial()->Mass();

      // Initialize physics parameters from SDF (with defaults from physics_types.hpp)
      this->physics_params_.mass = this->mass_;
      if (_sdf->HasElement("actuator_tau")) {
        this->physics_params_.actuator_tau = _sdf->Get<double>("actuator_tau");
      }
      if (_sdf->HasElement("actuator_tau_up")) {
        this->physics_params_.actuator_tau_up = _sdf->Get<double>("actuator_tau_up");
      }
      if (_sdf->HasElement("actuator_tau_down")) {
        this->physics_params_.actuator_tau_down = _sdf->Get<double>("actuator_tau_down");
      }
      if (_sdf->HasElement("max_accel")) {
        this->physics_params_.max_accel = _sdf->Get<double>("max_accel");
      }
      if (_sdf->HasElement("drag_coeff_lin")) {
        this->physics_params_.drag_coeff_lin = _sdf->Get<double>("drag_coeff_lin");
      }
      if (_sdf->HasElement("drag_coeff_quad")) {
        this->physics_params_.drag_coeff_quad = _sdf->Get<double>("drag_coeff_quad");
      }
      if (_sdf->HasElement("drag_speed_threshold")) {
        this->physics_params_.drag_speed_threshold = _sdf->Get<double>("drag_speed_threshold");
      }

      // Load initial wind parameters from SDF - opt. fallback
      // These can be overridden by /wind/velocity and /wind/force topics
      if (_sdf->HasElement("wind_velocity_x")) {
        this->wind_env_.vx = _sdf->Get<double>("wind_velocity_x");
      }
      if (_sdf->HasElement("wind_velocity_y")) {
        this->wind_env_.vy = _sdf->Get<double>("wind_velocity_y");
      }
      if (_sdf->HasElement("wind_accel_bias_x")) {
        this->wind_env_.ax_bias = _sdf->Get<double>("wind_accel_bias_x");
      }
      if (_sdf->HasElement("wind_accel_bias_y")) {
        this->wind_env_.ay_bias = _sdf->Get<double>("wind_accel_bias_y");
      }

      // Get target altitude from SDF (for altitude lane separation)
      if (_sdf->HasElement("target_altitude"))
      {
        this->target_altitude_ = _sdf->Get<double>("target_altitude");
      }

      // Get sensor noise parameters from SDF
      // Position noise: Typical values 0.01-0.05m for optical flow, 0.1-1.0m for GPS
      if (_sdf->HasElement("position_noise_std")) {
        this->position_noise_std_ = _sdf->Get<double>("position_noise_std");
      }
      // Velocity noise: Typical values 0.02-0.1 m/s for IMU-based estimation
      if (_sdf->HasElement("velocity_noise_std")) {
        this->velocity_noise_std_ = _sdf->Get<double>("velocity_noise_std");
      }

      // Initialize random number generator
      std::random_device rd;
      this->rng_ = std::mt19937(rd());
      this->position_noise_dist_ = std::normal_distribution<double>(0.0, this->position_noise_std_);
      this->velocity_noise_dist_ = std::normal_distribution<double>(0.0, this->velocity_noise_std_);

      // ===== Motor Model Feature Flag =====
      // Load use_motor_model flag from SDF (default: false for backward compatibility)
      if (_sdf->HasElement("use_motor_model")) {
        this->use_motor_model_ = _sdf->Get<bool>("use_motor_model");
      }

      // Get namespace from SDF (e.g., "agent_0" for multi-agent support)
      if (_sdf->HasElement("namespace"))
      {
        this->namespace_ = _sdf->Get<std::string>("namespace");
      }
      else
      {
        // Fallback: use model name as namespace
        this->namespace_ = this->model_->GetName();
      }

      // Initialize ROS2 node
      this->ros_node_ = gazebo_ros::Node::Get(_sdf);

      RCLCPP_INFO(this->ros_node_->get_logger(),
                  "SimpleDronePlugin loading for namespace: %s, mass: %.2f kg, target_altitude: %.2f m, "
                  "physics[tau_up=%.3f, tau_down=%.3f, cd_lin=%.3f, cd_quad=%.3f], "
                  "wind[vx=%.2f, vy=%.2f, ax_bias=%.3f, ay_bias=%.3f]",
                  this->namespace_.c_str(), this->mass_, this->target_altitude_,
                  this->physics_params_.actuator_tau_up, this->physics_params_.actuator_tau_down,
                  this->physics_params_.drag_coeff_lin, this->physics_params_.drag_coeff_quad,
                  this->wind_env_.vx, this->wind_env_.vy,
                  this->wind_env_.ax_bias, this->wind_env_.ay_bias);

      // Subscribe to acceleration commands from agent_controller_node
      std::string cmd_accel_topic = "/" + this->namespace_ + "/cmd_accel";
      this->cmd_accel_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
        cmd_accel_topic, 10,
        std::bind(&SimpleDronePlugin::OnCmdAccel, this, std::placeholders::_1));

      // Subscribe to wind velocity (for drag calculation with relative airspeed)
      // Per-drone wind topics for spatial coherence support: /{namespace}/wind/velocity
      // Falls back to global /wind/velocity if per-drone topic not available
      std::string wind_velocity_topic = "/" + this->namespace_ + "/wind/velocity";
      this->wind_velocity_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
        wind_velocity_topic, 10,
        std::bind(&SimpleDronePlugin::OnWindVelocity, this, std::placeholders::_1));

      // Also subscribe to global wind velocity as fallback (lower priority)
      this->wind_velocity_sub_global_ = this->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/wind/velocity", 10,
        std::bind(&SimpleDronePlugin::OnWindVelocity, this, std::placeholders::_1));

      // Subscribe to wind force (for constant acceleration bias)
      // Per-drone wind force topics
      std::string wind_force_topic = "/" + this->namespace_ + "/wind/force";
      this->wind_force_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
        wind_force_topic, 10,
        std::bind(&SimpleDronePlugin::OnWindForce, this, std::placeholders::_1));

      // Global wind force fallback
      this->wind_force_sub_global_ = this->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/wind/force", 10,
        std::bind(&SimpleDronePlugin::OnWindForce, this, std::placeholders::_1));

      // Publish odometry at Gazebo physics rate (~1kHz)
      std::string odom_topic = "/" + this->namespace_ + "/odom";
      // Use SensorDataQoS to match subscribers in controllers/metrics nodes
      this->odom_pub_ = this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS());

      // Connect to Gazebo world update event (called every physics step)
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SimpleDronePlugin::OnUpdate, this));

      // ===== Initialize Motor Model (if enabled) =====
      if (this->use_motor_model_) {
        // Create motor model with sim_cf2 parameters
        auto motor_params = agent_control_pkg::core::MotorParams::sim_cf2();
        this->motor_model_ = std::make_unique<agent_control_pkg::core::CrazyflieMotorModel>(motor_params);

        // Create mixer
        auto mixer_config = agent_control_pkg::core::MixerConfig::fromMotorParams(motor_params);
        this->mixer_ = std::make_unique<agent_control_pkg::core::XConfigMixer>(mixer_config);

        // Create accel-to-thrust converter
        this->accel_to_thrust_ = std::make_unique<agent_control_pkg::core::AccelToThrustSimple>(
          this->mass_, 9.81);

        // Initialize motors to hover state
        this->motor_model_->setHover(this->mass_, 9.81);

        // Get rotor joint references for visual spinning
        const std::array<std::string, 4> joint_names = {
          "rotor_fr_joint", "rotor_fl_joint", "rotor_bl_joint", "rotor_br_joint"
        };

        for (int i = 0; i < 4; ++i) {
          // Try direct joint lookup first
          this->rotor_joints_[i] = this->model_->GetJoint(joint_names[i]);

          // If not found, try with nested model prefix (for <include> usage)
          if (!this->rotor_joints_[i]) {
            auto joints = this->model_->GetJoints();
            for (const auto& joint : joints) {
              if (joint->GetName().find(joint_names[i]) != std::string::npos) {
                this->rotor_joints_[i] = joint;
                break;
              }
            }
          }

          if (this->rotor_joints_[i]) {
            RCLCPP_DEBUG(this->ros_node_->get_logger(),
                         "Found rotor joint: %s", joint_names[i].c_str());
          }
        }

        RCLCPP_INFO(this->ros_node_->get_logger(),
          "Motor model ENABLED: omega_max=%.0f rad/s, k_thrust=%.2e, k_moment=%.4f, hover_omega=%.0f rad/s",
          motor_params.omega_max, motor_params.k_thrust, motor_params.k_moment,
          motor_params.hoverOmega(this->mass_));
      }

      RCLCPP_INFO(this->ros_node_->get_logger(),
                  "SimpleDronePlugin loaded successfully! (motor_model=%s)",
                  this->use_motor_model_ ? "true" : "false");
    }

  private:
    /**
     * @brief Callback for acceleration commands from ROS2
     * @param msg Vector3 containing desired X, Y, Z accelerations (m/s²)
     *
     * Stores the commanded acceleration for use in the next physics update.
     * Z component is ignored as altitude is automatically maintained.
     */
    void OnCmdAccel(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
      // Store commanded acceleration for X-Y plane motion
      this->cmd_accel_x_ = msg->x;
      this->cmd_accel_y_ = msg->y;
      // Z locked for 2D simulation (altitude maintained separately)
      this->cmd_accel_z_ = 0.0;
    }

    /**
     * @brief Callback for wind velocity
     * @param msg Vector3 containing wind velocity in m/s
     *
     * Updates the wind environment for drag calculation (relative airspeed).
     * This enables time-varying wind scenarios (sinusoid, step, gust).
     *
     * Topic: /wind/velocity
     * Message format: geometry_msgs/Vector3
     *   - x: Wind velocity X component [m/s]
     *   - y: Wind velocity Y component [m/s]
     *   - z: Wind velocity Z component [m/s] (unused for 2D)
     */
    void OnWindVelocity(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
      this->wind_env_.vx = msg->x;
      this->wind_env_.vy = msg->y;
      // Drag force is computed as F_drag = -cd * (v - v_wind)
      // Updating wind velocity affects relative airspeed in physics core
    }

    /**
     * @brief Callback for wind force (constant acceleration bias)
     * @param msg Vector3 containing wind force in Newtons (N)
     *
     * Stores the current wind disturbance force to be applied in physics update.
     * Also converts force to acceleration bias for physics core.
     *
     * Topic: /wind/force
     * Message format: geometry_msgs/Vector3
     *   - x: Wind force X component [N]
     *   - y: Wind force Y component [N]
     *   - z: Wind force Z component [N] (unused for 2D)
     */
    void OnWindForce(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
      this->wind_force_x_ = msg->x;
      this->wind_force_y_ = msg->y;
      this->wind_force_z_ = msg->z;

      // Convert wind force to acceleration bias for physics core
      // F = m * a  =>  a = F / m
      this->wind_env_.ax_bias = msg->x / this->mass_;
      this->wind_env_.ay_bias = msg->y / this->mass_;
    }

    /**
     * @brief Main physics update loop (called every Gazebo timestep)
     *
     * This function applies forces to the drone model based on:
     * 1. Advanced 2D dynamics: actuator lag + drag blend + wind bias
     * 2. Automatic gravity compensation and altitude hold at Z=0.5m
     *
     * Update rate: Synchronized with Gazebo physics (typically ~1kHz)
     */
    void OnUpdate()
    {
      if (!this->link_)
        return;

      // Get current time and compute dt
      rclcpp::Time now = this->ros_node_->get_clock()->now();
      double dt = 0.001;  // Default 1ms (Gazebo physics rate)
      if (!this->first_physics_update_) {
        dt = (now - this->last_physics_update_time_).seconds();
        dt = std::clamp(dt, 0.0001, 0.01);  // Clamp to reasonable range
      } else {
        this->first_physics_update_ = false;
      }
      this->last_physics_update_time_ = now;

      // ===== Branch: Motor Model vs Legacy F=ma =====
      if (this->use_motor_model_ && this->motor_model_) {
        applyMotorPhysics(dt);
      } else {
        applyLegacyPhysics(dt);
      }

      // Publish current state to ROS2 (odometry feedback)
      this->PublishOdometry();
    }

    /**
     * @brief In the current world configuration (use_motor_model=true), the simulation updates drone dynamics via applyMotorPhysics; applyLegacyPhysics is only used as a fallback when the motor model is disabled or not initialized.
     * @param dt 
     */
    void applyLegacyPhysics(double dt)
    {
      // Get current velocity from Gazebo
      auto linear_vel = this->link_->WorldLinearVel();
      const double vx = linear_vel.X();
      const double vy = linear_vel.Y();

      // Compute advanced 2D physics using core library
      // This includes: actuator lag, drag blend, wind bias
      double ax_total, ay_total;
      agent_control_pkg::core::DronePhysicsCore::computePhysicsStep(
        this->cmd_accel_x_,         // Commanded acceleration X
        this->cmd_accel_y_,         // Commanded acceleration Y
        vx, vy,                      // Current velocity
        this->actuator_state_,       // Actuator filter state (updated in-place)
        this->wind_env_,             // Wind environment
        this->physics_params_,       // Physics parameters
        dt,                          // Time step
        ax_total, ay_total,          // Output: total acceleration
        &this->physics_diag_         // Optional diagnostics
      );

      // Convert acceleration to force: F = m * a
      ignition::math::Vector3d force(
        this->mass_ * ax_total,
        this->mass_ * ay_total,
        0.0
      );

      // Z-axis altitude hold system:
      // 1. Cancel gravity: F_gravity_comp = m * g (9.81 m/s²)
      // 2. Add proportional control to maintain target altitude (from SDF parameter)
      auto pose = this->link_->WorldPose();
      double current_z = pose.Pos().Z();
      double target_z = this->target_altitude_;  // Target altitude from SDF (meters)
      double z_error = target_z - current_z;
      auto linear_vel_z = this->link_->WorldLinearVel();
      double vel_z = linear_vel_z.Z();

      // Total Z force: gravity compensation + P-controller
      // Fz = m * (g + Kp * error - Kd * vel_z)
      double altitude_term = this->altitude_kp_ * z_error - this->altitude_kd_ * vel_z;
      double z_force = this->mass_ * (9.81 + altitude_term);

      force.Z(z_force);

      // Apply total force to drone's center of mass
      this->link_->AddForce(force);

      // Attitude stabilization: keep roll/pitch near zero, damp angular rates
      auto angular_vel = this->link_->WorldAngularVel();
      ignition::math::Vector3d attitude_error(pose.Rot().Roll(), pose.Rot().Pitch(), 0.0);
      ignition::math::Vector3d stabilization_torque(
        -this->attitude_stiffness_ * attitude_error.X(),
        -this->attitude_stiffness_ * attitude_error.Y(),
        -this->yaw_damping_ * angular_vel.Z());

      stabilization_torque -= angular_vel * this->angular_damping_;
      this->link_->AddTorque(stabilization_torque);
    }

    /**
     * @brief Apply motor-based physics (sim_cf2 motor model)
     * @param dt Time step [s]
     *
     * Phase 3 implementation: Collective thrust only, no attitude mixing.
     * Full cascade control will be added in Phase 5.
     */
    void applyMotorPhysics(double dt)
    {
      auto pose = this->link_->WorldPose();
      auto linear_vel = this->link_->WorldLinearVel();

      // Altitude PD control to compute required Z acceleration
      double current_z = pose.Pos().Z();
      double z_error = this->target_altitude_ - current_z;
      double vel_z = linear_vel.Z();
      double az_cmd = this->altitude_kp_ * z_error - this->altitude_kd_ * vel_z;

      // Convert Z acceleration to collective thrust: T = m*(g + az_cmd)
      double thrust_total = this->accel_to_thrust_->accelToThrust(az_cmd);

      // Use collective-only mixing (equal distribution to all motors)
      // Full attitude mixing will be added in Phase 5
      auto omega_cmds = this->mixer_->mixCollectiveOnly(thrust_total);
      this->motor_model_->setAllOmegaCmd(omega_cmds);

      // Update motor dynamics (first-order lag with asymmetric tau)
      this->motor_model_->update(dt);

      // Apply thrust forces at each rotor position
      auto world_rot = pose.Rot();
      for (int i = 0; i < 4; ++i) {
        const auto& motor = this->motor_model_->getMotor(i);
        const auto& pos = this->motor_model_->getRotorPosition(i);

        // Thrust force (upward in body frame)
        ignition::math::Vector3d thrust_body(0, 0, motor.thrust);

        // Transform to world frame
        ignition::math::Vector3d thrust_world = world_rot.RotateVector(thrust_body);

        // Apply force at rotor position (relative to body CoM)
        ignition::math::Vector3d rotor_pos(pos[0], pos[1], pos[2]);
        this->link_->AddForceAtRelativePosition(thrust_world, rotor_pos);
      }

      // Apply yaw torque from motor differential (CCW motors vs CW motors)
      double yaw_torque = this->motor_model_->getTotalYawTorque();
      ignition::math::Vector3d yaw_torque_body(0, 0, yaw_torque);
      ignition::math::Vector3d yaw_torque_world = world_rot.RotateVector(yaw_torque_body);
      this->link_->AddTorque(yaw_torque_world);

      // Attitude stabilization (same as legacy for XY control)
      // This provides roll/pitch stability until cascade control is implemented
      auto angular_vel = this->link_->WorldAngularVel();
      ignition::math::Vector3d attitude_error(pose.Rot().Roll(), pose.Rot().Pitch(), 0.0);
      ignition::math::Vector3d stabilization_torque(
        -this->attitude_stiffness_ * attitude_error.X(),
        -this->attitude_stiffness_ * attitude_error.Y(),
        -this->yaw_damping_ * angular_vel.Z());
      stabilization_torque -= angular_vel * this->angular_damping_;
      this->link_->AddTorque(stabilization_torque);

      // Apply XY forces from legacy physics (for 2D control)
      // This allows existing PID/Fuzzy controllers to work unchanged
      double vx = linear_vel.X();
      double vy = linear_vel.Y();
      double ax_total, ay_total;
      agent_control_pkg::core::DronePhysicsCore::computePhysicsStep(
        this->cmd_accel_x_, this->cmd_accel_y_,
        vx, vy,
        this->actuator_state_,
        this->wind_env_,
        this->physics_params_,
        dt,
        ax_total, ay_total,
        &this->physics_diag_
      );

      ignition::math::Vector3d xy_force(
        this->mass_ * ax_total,
        this->mass_ * ay_total,
        0.0
      );
      this->link_->AddForce(xy_force);

      // Update rotor visual spinning
      updateRotorVisuals();
    }

    /**
     * @brief Update rotor joint velocities for visual spinning
     */
    void updateRotorVisuals()
    {
      if (!this->use_motor_model_ || !this->motor_model_) return;

      for (int i = 0; i < 4; ++i) {
        if (!this->rotor_joints_[i]) continue;

        const auto& motor = this->motor_model_->getMotor(i);
        // Get rotor direction: CW motors spin negative, CCW positive
        int dir = this->motor_model_->getRotorDirection(i);
        double visual_vel = dir * motor.omega;

        // Set joint velocity for visual rotation
        this->rotor_joints_[i]->SetVelocity(0, visual_vel);
      }
    }

    /**
     * @brief Publish full state odometry message
     *
     * Publishes position, orientation, linear velocity, and angular velocity
     * from Gazebo physics simulation to ROS2. This provides feedback to the
     * agent_controller_node for closed-loop control.
     *
     * Message: nav_msgs/Odometry
     *   - header.frame_id: "odom" (world frame)
     *   - child_frame_id: "base_link" (drone body frame)
     *   - pose: 3D position + quaternion orientation
     *   - twist: 3D linear + angular velocities
     */
    void PublishOdometry()
    {
      // Get current state from Gazebo physics engine
      auto pose = this->link_->WorldPose();
      auto linear_vel = this->link_->WorldLinearVel();
      auto angular_vel = this->link_->WorldAngularVel();

      // Generate sensor noise (if enabled)
      double pos_noise_x = 0.0, pos_noise_y = 0.0;
      double vel_noise_x = 0.0, vel_noise_y = 0.0;
      if (this->position_noise_std_ > 0.0) {
        pos_noise_x = this->position_noise_dist_(this->rng_);
        pos_noise_y = this->position_noise_dist_(this->rng_);
      }
      if (this->velocity_noise_std_ > 0.0) {
        vel_noise_x = this->velocity_noise_dist_(this->rng_);
        vel_noise_y = this->velocity_noise_dist_(this->rng_);
      }

      // Construct odometry message
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = this->ros_node_->get_clock()->now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";

      // Position with noise (meters)
      odom.pose.pose.position.x = pose.Pos().X() + pos_noise_x;
      odom.pose.pose.position.y = pose.Pos().Y() + pos_noise_y;
      odom.pose.pose.position.z = pose.Pos().Z();  // No noise on Z (altitude lock)

      // Orientation (quaternion) - no noise
      odom.pose.pose.orientation.x = pose.Rot().X();
      odom.pose.pose.orientation.y = pose.Rot().Y();
      odom.pose.pose.orientation.z = pose.Rot().Z();
      odom.pose.pose.orientation.w = pose.Rot().W();

      // Linear velocity with noise (m/s)
      odom.twist.twist.linear.x = linear_vel.X() + vel_noise_x;
      odom.twist.twist.linear.y = linear_vel.Y() + vel_noise_y;
      odom.twist.twist.linear.z = linear_vel.Z();  // No noise on Z

      // Angular velocity (rad/s) - no noise
      odom.twist.twist.angular.x = angular_vel.X();
      odom.twist.twist.angular.y = angular_vel.Y();
      odom.twist.twist.angular.z = angular_vel.Z();

      // Publish to ROS2
      this->odom_pub_->publish(odom);
    }

    // ===== Gazebo Physics Objects =====
    physics::ModelPtr model_;  ///< Pointer to the drone model
    physics::LinkPtr link_;    ///< Pointer to base_link (main body)

    // ===== ROS2 Interface =====
    gazebo_ros::Node::SharedPtr ros_node_;  ///< ROS2 node handle
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmd_accel_sub_;  ///< Acceleration command subscriber
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_velocity_sub_;  ///< Per-drone wind velocity subscriber
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_velocity_sub_global_;  ///< Global wind velocity fallback
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_force_sub_;  ///< Per-drone wind force subscriber
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_force_sub_global_;  ///< Global wind force fallback
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;  ///< Odometry publisher

    // ===== Gazebo Event Connection =====
    event::ConnectionPtr update_connection_;  ///< Connection to world update event

    // ===== Parameters =====
    std::string namespace_;  ///< ROS2 namespace (e.g., "agent_0")
    double mass_;            ///< Drone mass in kg (from URDF/SDF inertial)

    // ===== Control State =====
    double cmd_accel_x_ = 0.0;  ///< Commanded X acceleration (m/s²)
    double cmd_accel_y_ = 0.0;  ///< Commanded Y acceleration (m/s²)
    double cmd_accel_z_ = 0.0;  ///< Commanded Z acceleration (unused, locked)

    // ===== Wind Disturbance State =====
    double wind_force_x_ = 0.0;  ///< Wind force X component (N)
    double wind_force_y_ = 0.0;  ///< Wind force Y component (N)
    double wind_force_z_ = 0.0;  ///< Wind force Z component (N, unused)

    // ===== Advanced Physics Core =====
    agent_control_pkg::core::PhysicsParams physics_params_;      ///< Physics model parameters
    agent_control_pkg::core::ActuatorState actuator_state_;      ///< Actuator filter state
    agent_control_pkg::core::WindEnvironment wind_env_;          ///< Wind environment
    agent_control_pkg::core::PhysicsDiagnostics physics_diag_;   ///< Physics diagnostics

    // ===== Attitude/Altitude Stabilization Parameters =====
    // Note: Linear drag is handled by physics_params_ (cd_lin, cd_quad)
    // These parameters only control attitude (roll/pitch/yaw) and altitude (Z)
    double angular_damping_ = 0.2;       ///< Angular velocity damping coefficient
    double attitude_stiffness_ = 4.0;    ///< Restoring torque for roll/pitch
    double yaw_damping_ = 0.1;           ///< Yaw damping to prevent spinning
    double altitude_kp_ = 12.0;          ///< Altitude proportional gain (aggressive Z response)
    double altitude_kd_ = 4.0;           ///< Altitude velocity damping gain (critical damping)

    // ===== Physics Update Timing =====
    rclcpp::Time last_physics_update_time_;  ///< Last physics update timestamp
    bool first_physics_update_ = true;       ///< Flag for first update

    // ===== Target Altitude (from SDF) =====
    double target_altitude_ = 0.5;  ///< Target altitude for this drone (meters), configurable from world file

    // ===== Sensor Noise Parameters (for realistic simulation) =====
    // These enable testing Type-2 Fuzzy's uncertainty handling capability
    double position_noise_std_ = 0.0;  ///< Position noise std dev [m] (0 = no noise)
    double velocity_noise_std_ = 0.0;  ///< Velocity noise std dev [m/s] (0 = no noise)
    mutable std::mt19937 rng_;  ///< Random number generator
    mutable std::normal_distribution<double> position_noise_dist_;  ///< Position noise distribution
    mutable std::normal_distribution<double> velocity_noise_dist_;  ///< Velocity noise distribution

    // ===== Motor Model (sim_cf2 parameters) =====
    bool use_motor_model_ = false;  ///< Feature flag: true = motor model, false = legacy F=ma
    std::unique_ptr<agent_control_pkg::core::CrazyflieMotorModel> motor_model_;  ///< Motor dynamics
    std::unique_ptr<agent_control_pkg::core::XConfigMixer> mixer_;  ///< Thrust allocation mixer
    std::unique_ptr<agent_control_pkg::core::AccelToThrustSimple> accel_to_thrust_;  ///< Accel-to-thrust converter
    std::array<physics::JointPtr, 4> rotor_joints_;  ///< Rotor joint pointers for visual spinning
  };

  // Register this plugin with Gazebo
  GZ_REGISTER_MODEL_PLUGIN(SimpleDronePlugin)
}  // namespace gazebo
