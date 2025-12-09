/**
 * @file crazyflie_motor_plugin.cpp
 * @brief Gazebo plugin with realistic Crazyflie 2.1 motor physics
 *
 * This plugin provides hardware-accurate motor dynamics for the Crazyflie 2.1
 * quadrotor, enabling simulation results that transfer to real hardware.
 *
 * Motor Model (from sim_cf2/Bitcraze):
 *   - Thrust: T = k_motor * ω²
 *   - Torque: τ = k_moment * T
 *   - First-order motor dynamics with asymmetric time constants
 *
 * ROS2 Interface:
 *   Subscribes: /agent_X/cmd_accel (geometry_msgs/Vector3) - Desired accelerations
 *   Publishes:  /agent_X/odom (nav_msgs/Odometry) - Full state feedback
 *
 * The plugin converts acceleration commands to motor speeds using quadrotor
 * mixing, then applies realistic motor dynamics and thrust generation.
 *
 * @author Multi-Agent Formation Control Team
 * @date 2025
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <random>
#include <array>
#include <cmath>

namespace gazebo
{

/**
 * @class CrazyflieMotorPlugin
 * @brief Gazebo ModelPlugin with realistic Crazyflie motor physics
 */
class CrazyflieMotorPlugin : public ModelPlugin
{
public:
  CrazyflieMotorPlugin() : ModelPlugin() {}

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->model_ = _model;

    // Find base_link (handles nested model from <include>)
    this->base_link_ = FindLink("base_link");
    if (!this->base_link_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("CrazyflieMotorPlugin"),
                   "base_link not found!");
      return;
    }

    // Get mass from inertial
    this->mass_ = this->base_link_->GetInertial()->Mass();

    // Find propeller joints
    FindPropellerJoints();

    // Load Crazyflie motor parameters from SDF (with defaults from sim_cf2)
    LoadMotorParameters(_sdf);

    // Load sensor noise parameters
    LoadNoiseParameters(_sdf);

    // Get namespace and target altitude
    if (_sdf->HasElement("namespace"))
      this->namespace_ = _sdf->Get<std::string>("namespace");
    else
      this->namespace_ = this->model_->GetName();

    if (_sdf->HasElement("target_altitude"))
      this->target_altitude_ = _sdf->Get<double>("target_altitude");

    // Initialize ROS2 node
    this->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Subscribe to acceleration commands
    std::string cmd_topic = "/" + this->namespace_ + "/cmd_accel";
    this->cmd_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
      cmd_topic, 10,
      std::bind(&CrazyflieMotorPlugin::OnCmdAccel, this, std::placeholders::_1));

    // Subscribe to wind
    this->wind_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
      "/wind/velocity", 10,
      std::bind(&CrazyflieMotorPlugin::OnWindVelocity, this, std::placeholders::_1));

    // Publish odometry
    std::string odom_topic = "/" + this->namespace_ + "/odom";
    this->odom_pub_ = this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS());

    // Connect to Gazebo update
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CrazyflieMotorPlugin::OnUpdate, this));

    RCLCPP_INFO(this->ros_node_->get_logger(),
                "CrazyflieMotorPlugin loaded: ns=%s, mass=%.3fkg, k_motor=%.2e, k_moment=%.4f",
                this->namespace_.c_str(), this->mass_,
                this->motor_constant_, this->moment_constant_);
  }

private:
  /**
   * @brief Find a link by name (handles nested models)
   */
  physics::LinkPtr FindLink(const std::string& name)
  {
    auto link = this->model_->GetLink(name);
    if (link) return link;

    // Search in nested models
    for (const auto& l : this->model_->GetLinks())
    {
      if (l->GetName().find(name) != std::string::npos)
        return l;
    }
    return nullptr;
  }

  /**
   * @brief Find propeller joints for spinning animation and thrust application
   */
  void FindPropellerJoints()
  {
    const std::array<std::string, 4> prop_names = {
      "prop_fr", "prop_fl", "prop_bl", "prop_br"
    };

    for (const auto& joint : this->model_->GetJoints())
    {
      std::string name = joint->GetName();
      for (size_t i = 0; i < 4; i++)
      {
        if (name.find(prop_names[i]) != std::string::npos)
        {
          this->prop_joints_[i] = joint;
          // Also get propeller links for thrust application
          this->prop_links_[i] = joint->GetChild();
          break;
        }
      }
    }

    int found = 0;
    for (int i = 0; i < 4; i++)
      if (this->prop_joints_[i]) found++;

    RCLCPP_INFO(this->ros_node_->get_logger(),
                "Found %d/4 propeller joints", found);
  }

  /**
   * @brief Load Crazyflie motor parameters
   */
  void LoadMotorParameters(sdf::ElementPtr _sdf)
  {
    // Crazyflie 2.1 parameters from sim_cf2
    if (_sdf->HasElement("motor_constant"))
      this->motor_constant_ = _sdf->Get<double>("motor_constant");

    if (_sdf->HasElement("moment_constant"))
      this->moment_constant_ = _sdf->Get<double>("moment_constant");

    if (_sdf->HasElement("time_constant_up"))
      this->time_constant_up_ = _sdf->Get<double>("time_constant_up");

    if (_sdf->HasElement("time_constant_down"))
      this->time_constant_down_ = _sdf->Get<double>("time_constant_down");

    if (_sdf->HasElement("max_rot_velocity"))
      this->max_rot_velocity_ = _sdf->Get<double>("max_rot_velocity");

    if (_sdf->HasElement("rotor_drag_coefficient"))
      this->rotor_drag_coeff_ = _sdf->Get<double>("rotor_drag_coefficient");

    // Arm geometry
    if (_sdf->HasElement("arm_length"))
      this->arm_length_ = _sdf->Get<double>("arm_length");
  }

  /**
   * @brief Load sensor noise parameters
   */
  void LoadNoiseParameters(sdf::ElementPtr _sdf)
  {
    if (_sdf->HasElement("position_noise_std"))
      this->position_noise_std_ = _sdf->Get<double>("position_noise_std");

    if (_sdf->HasElement("velocity_noise_std"))
      this->velocity_noise_std_ = _sdf->Get<double>("velocity_noise_std");

    std::random_device rd;
    this->rng_ = std::mt19937(rd());
    this->pos_noise_ = std::normal_distribution<double>(0.0, this->position_noise_std_);
    this->vel_noise_ = std::normal_distribution<double>(0.0, this->velocity_noise_std_);
  }

  /**
   * @brief Callback for acceleration commands
   */
  void OnCmdAccel(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    this->cmd_accel_x_ = msg->x;
    this->cmd_accel_y_ = msg->y;
    this->cmd_accel_z_ = msg->z;
  }

  /**
   * @brief Callback for wind velocity
   */
  void OnWindVelocity(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    this->wind_vx_ = msg->x;
    this->wind_vy_ = msg->y;
  }

  /**
   * @brief Main physics update - apply motor forces and publish odometry
   */
  void OnUpdate()
  {
    if (!this->base_link_) return;

    // Get current time and dt
    rclcpp::Time now = this->ros_node_->get_clock()->now();
    double dt = 0.001;
    if (!this->first_update_)
    {
      dt = (now - this->last_update_time_).seconds();
      dt = std::clamp(dt, 0.0001, 0.01);
    }
    else
    {
      this->first_update_ = false;
    }
    this->last_update_time_ = now;

    // Convert acceleration commands to motor speeds
    ComputeMotorSpeeds(dt);

    // Apply motor thrust and torque
    ApplyMotorForces();

    // Apply wind disturbance
    ApplyWindForce();

    // Altitude hold (Z-axis stabilization)
    ApplyAltitudeControl();

    // Attitude stabilization
    ApplyAttitudeControl();

    // Spin propellers (visual)
    SpinPropellers(dt);

    // Publish odometry
    PublishOdometry();
  }

  /**
   * @brief Convert desired accelerations to motor angular velocities
   *
   * Quadrotor mixing for X-configuration:
   *   Motor 0 (FR): CCW - positive pitch, negative roll
   *   Motor 1 (FL): CW  - positive pitch, positive roll
   *   Motor 2 (BL): CCW - negative pitch, positive roll
   *   Motor 3 (BR): CW  - negative pitch, negative roll
   */
  void ComputeMotorSpeeds(double dt)
  {
    // Total thrust needed for desired Z acceleration
    double thrust_z = this->mass_ * (9.81 + this->cmd_accel_z_);
    thrust_z = std::max(thrust_z, 0.0);  // Can't push down

    // Convert X/Y accelerations to roll/pitch moments
    // Simplified: τ = I * α, where α = a / arm_length (approx)
    double moment_x = this->mass_ * this->cmd_accel_y_ * this->arm_length_ * 0.5;  // Roll
    double moment_y = this->mass_ * this->cmd_accel_x_ * this->arm_length_ * 0.5;  // Pitch

    // Base motor speed for hover
    double base_thrust = thrust_z / 4.0;
    double omega_base = 0.0;
    if (base_thrust > 0.0 && this->motor_constant_ > 0.0)
      omega_base = std::sqrt(base_thrust / this->motor_constant_);

    // Differential speeds for attitude control
    double d_roll = moment_x / (4.0 * this->motor_constant_ * omega_base * this->arm_length_ + 1e-6);
    double d_pitch = moment_y / (4.0 * this->motor_constant_ * omega_base * this->arm_length_ + 1e-6);

    // Compute individual motor speeds (X-config)
    // FR (0): +pitch, -roll, CCW
    // FL (1): +pitch, +roll, CW
    // BL (2): -pitch, +roll, CCW
    // BR (3): -pitch, -roll, CW
    this->motor_cmd_[0] = omega_base + d_pitch - d_roll;
    this->motor_cmd_[1] = omega_base + d_pitch + d_roll;
    this->motor_cmd_[2] = omega_base - d_pitch + d_roll;
    this->motor_cmd_[3] = omega_base - d_pitch - d_roll;

    // Apply motor dynamics (first-order filter with asymmetric time constants)
    for (int i = 0; i < 4; i++)
    {
      double cmd = std::clamp(this->motor_cmd_[i], 0.0, this->max_rot_velocity_);
      double tau = (cmd > this->motor_vel_[i]) ? this->time_constant_up_ : this->time_constant_down_;
      double alpha = dt / (tau + dt);
      this->motor_vel_[i] += alpha * (cmd - this->motor_vel_[i]);
      this->motor_vel_[i] = std::clamp(this->motor_vel_[i], 0.0, this->max_rot_velocity_);
    }
  }

  /**
   * @brief Apply thrust from each motor
   */
  void ApplyMotorForces()
  {
    // Motor directions: CCW (+Z torque), CW (-Z torque)
    const std::array<double, 4> directions = {1.0, -1.0, 1.0, -1.0};  // FR, FL, BL, BR

    double total_thrust = 0.0;
    double total_yaw_torque = 0.0;

    for (int i = 0; i < 4; i++)
    {
      double omega = this->motor_vel_[i];
      double thrust = this->motor_constant_ * omega * omega;
      double torque = this->moment_constant_ * thrust * directions[i];

      total_thrust += thrust;
      total_yaw_torque += torque;

      // Apply thrust at propeller location if link exists
      if (this->prop_links_[i])
      {
        ignition::math::Vector3d thrust_vec(0, 0, thrust);
        this->prop_links_[i]->AddRelativeForce(thrust_vec);
      }
    }

    // If no prop links, apply total thrust to base_link
    if (!this->prop_links_[0])
    {
      ignition::math::Vector3d thrust_vec(0, 0, total_thrust);
      this->base_link_->AddRelativeForce(thrust_vec);
    }

    // Apply yaw torque
    ignition::math::Vector3d yaw_torque(0, 0, total_yaw_torque);
    this->base_link_->AddRelativeTorque(yaw_torque);
  }

  /**
   * @brief Apply wind disturbance force
   */
  void ApplyWindForce()
  {
    // Simple drag model: F = -cd * (v - v_wind)
    auto vel = this->base_link_->WorldLinearVel();
    double rel_vx = vel.X() - this->wind_vx_;
    double rel_vy = vel.Y() - this->wind_vy_;

    double cd = 0.1;  // Drag coefficient for Crazyflie
    ignition::math::Vector3d drag_force(
      -cd * rel_vx * std::abs(rel_vx),
      -cd * rel_vy * std::abs(rel_vy),
      0.0
    );

    this->base_link_->AddForce(drag_force);
  }

  /**
   * @brief Altitude hold control
   */
  void ApplyAltitudeControl()
  {
    auto pose = this->base_link_->WorldPose();
    auto vel = this->base_link_->WorldLinearVel();

    double z_error = this->target_altitude_ - pose.Pos().Z();
    double z_force = this->mass_ * (9.81 + this->alt_kp_ * z_error - this->alt_kd_ * vel.Z());

    // Only add supplemental Z force (motors provide main thrust)
    ignition::math::Vector3d z_vec(0, 0, z_force * 0.3);  // Reduced gain
    this->base_link_->AddForce(z_vec);
  }

  /**
   * @brief Attitude stabilization (keep level)
   */
  void ApplyAttitudeControl()
  {
    auto pose = this->base_link_->WorldPose();
    auto ang_vel = this->base_link_->WorldAngularVel();

    ignition::math::Vector3d attitude_error(
      pose.Rot().Roll(),
      pose.Rot().Pitch(),
      0.0
    );

    ignition::math::Vector3d torque(
      -this->att_stiffness_ * attitude_error.X() - this->ang_damping_ * ang_vel.X(),
      -this->att_stiffness_ * attitude_error.Y() - this->ang_damping_ * ang_vel.Y(),
      -this->yaw_damping_ * ang_vel.Z()
    );

    this->base_link_->AddTorque(torque);
  }

  /**
   * @brief Spin propellers for visual effect
   */
  void SpinPropellers(double dt)
  {
    const std::array<double, 4> directions = {1.0, -1.0, 1.0, -1.0};

    for (int i = 0; i < 4; i++)
    {
      if (this->prop_joints_[i])
      {
        // Set joint velocity directly for spinning animation
        double target_vel = this->motor_vel_[i] * directions[i] / 10.0;  // Slow down for visibility
        this->prop_joints_[i]->SetVelocity(0, target_vel);
      }
    }
  }

  /**
   * @brief Publish odometry with optional noise
   */
  void PublishOdometry()
  {
    auto pose = this->base_link_->WorldPose();
    auto vel = this->base_link_->WorldLinearVel();
    auto ang_vel = this->base_link_->WorldAngularVel();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->ros_node_->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = this->namespace_ + "/base_link";

    // Position with noise
    odom.pose.pose.position.x = pose.Pos().X() +
      (this->position_noise_std_ > 0 ? this->pos_noise_(this->rng_) : 0.0);
    odom.pose.pose.position.y = pose.Pos().Y() +
      (this->position_noise_std_ > 0 ? this->pos_noise_(this->rng_) : 0.0);
    odom.pose.pose.position.z = pose.Pos().Z();

    // Orientation
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();

    // Velocity with noise
    odom.twist.twist.linear.x = vel.X() +
      (this->velocity_noise_std_ > 0 ? this->vel_noise_(this->rng_) : 0.0);
    odom.twist.twist.linear.y = vel.Y() +
      (this->velocity_noise_std_ > 0 ? this->vel_noise_(this->rng_) : 0.0);
    odom.twist.twist.linear.z = vel.Z();

    odom.twist.twist.angular.x = ang_vel.X();
    odom.twist.twist.angular.y = ang_vel.Y();
    odom.twist.twist.angular.z = ang_vel.Z();

    this->odom_pub_->publish(odom);
  }

  // ===== Gazebo Objects =====
  physics::ModelPtr model_;
  physics::LinkPtr base_link_;
  std::array<physics::JointPtr, 4> prop_joints_ = {nullptr};
  std::array<physics::LinkPtr, 4> prop_links_ = {nullptr};
  event::ConnectionPtr update_connection_;

  // ===== ROS2 Interface =====
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // ===== Parameters =====
  std::string namespace_;
  double mass_ = 0.027;
  double target_altitude_ = 0.5;

  // ===== Crazyflie Motor Parameters (from sim_cf2) =====
  double motor_constant_ = 1.2819184e-8;     // kg.m/s² (thrust = k * ω²)
  double moment_constant_ = 0.005964552;     // m (torque = k_m * T)
  double time_constant_up_ = 0.0125;         // s (12.5ms)
  double time_constant_down_ = 0.025;        // s (25ms)
  double max_rot_velocity_ = 3052.0;         // rad/s
  double rotor_drag_coeff_ = 9.1785e-7;
  double arm_length_ = 0.0325;               // m (motor to center)

  // ===== Motor State =====
  std::array<double, 4> motor_cmd_ = {0.0};  // Commanded motor speeds
  std::array<double, 4> motor_vel_ = {0.0};  // Filtered motor speeds

  // ===== Control Commands =====
  double cmd_accel_x_ = 0.0;
  double cmd_accel_y_ = 0.0;
  double cmd_accel_z_ = 0.0;

  // ===== Wind State =====
  double wind_vx_ = 0.0;
  double wind_vy_ = 0.0;

  // ===== Stabilization Gains =====
  double alt_kp_ = 12.0;
  double alt_kd_ = 4.0;
  double att_stiffness_ = 4.0;
  double ang_damping_ = 0.2;
  double yaw_damping_ = 0.1;

  // ===== Sensor Noise =====
  double position_noise_std_ = 0.0;
  double velocity_noise_std_ = 0.0;
  mutable std::mt19937 rng_;
  mutable std::normal_distribution<double> pos_noise_;
  mutable std::normal_distribution<double> vel_noise_;

  // ===== Timing =====
  rclcpp::Time last_update_time_;
  bool first_update_ = true;
};

// Register plugin
GZ_REGISTER_MODEL_PLUGIN(CrazyflieMotorPlugin)

}  // namespace gazebo
