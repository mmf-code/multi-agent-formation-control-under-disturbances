/**
 * @file simple_drone_plugin.cpp
 * @brief Gazebo plugin for 2D quadrotor dynamics simulation
 *
 * This plugin bridges ROS2 control commands with Gazebo physics simulation.
 * It implements a simplified 2D drone model with gravity compensation and
 * altitude locking, allowing X-Y plane motion control while maintaining
 * constant Z height.
 *
 * ROS2 Interface:
 *   Subscribes: /agent_X/cmd_accel (geometry_msgs/Vector3) - Acceleration commands
 *   Publishes:  /agent_X/odom (nav_msgs/Odometry) - Full state feedback
 *
 * Physics Model:
 *   - Applies forces directly: F = m * a_cmd (X-Y axes)
 *   - Z-axis: Gravity compensation + proportional altitude hold (target: 0.5m)
 *   - Update rate: Synchronized with Gazebo physics (default ~1kHz)
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

namespace gazebo
{
  /**
   * @class SimpleDronePlugin
   * @brief Gazebo ModelPlugin for simplified 2D quadrotor simulation
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
      this->link_ = this->model_->GetLink("base_link");

      if (!this->link_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("SimpleDronePlugin"),
                     "base_link not found in model!");
        return;
      }

      // Get drone mass from inertial properties (used for F=ma)
      this->mass_ = this->link_->GetInertial()->Mass();

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

      // Initialize ROS2 node (managed by gazebo_ros)
      this->ros_node_ = gazebo_ros::Node::Get(_sdf);

      RCLCPP_INFO(this->ros_node_->get_logger(),
                  "SimpleDronePlugin loading for namespace: %s, mass: %.2f kg",
                  this->namespace_.c_str(), this->mass_);

      // Subscribe to acceleration commands from agent_controller_node
      std::string cmd_accel_topic = "/" + this->namespace_ + "/cmd_accel";
      this->cmd_accel_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
        cmd_accel_topic, 10,
        std::bind(&SimpleDronePlugin::OnCmdAccel, this, std::placeholders::_1));

      // Publish odometry at Gazebo physics rate (~1kHz)
      std::string odom_topic = "/" + this->namespace_ + "/odom";
      this->odom_pub_ = this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, 10);

      // Connect to Gazebo world update event (called every physics step)
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SimpleDronePlugin::OnUpdate, this));

      RCLCPP_INFO(this->ros_node_->get_logger(),
                  "SimpleDronePlugin loaded successfully!");
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
     * @brief Main physics update loop (called every Gazebo timestep)
     *
     * This function applies forces to the drone model based on:
     * 1. X-Y acceleration commands from the controller (F = m*a)
     * 2. Automatic gravity compensation and altitude hold at Z=0.5m
     *
     * Update rate: Synchronized with Gazebo physics (typically ~1kHz)
     */
    void OnUpdate()
    {
      if (!this->link_)
        return;

      // Apply force = mass * acceleration for X-Y plane motion
      // This is the core of the 2D dynamics: F = ma
      ignition::math::Vector3d force(
        this->mass_ * this->cmd_accel_x_,
        this->mass_ * this->cmd_accel_y_,
        0.0
      );

      // Linear damping to suppress oscillations (acts like air drag)
      auto linear_vel = this->link_->WorldLinearVel();
      force -= linear_vel * this->linear_damping_;

      // Z-axis altitude hold system:
      // 1. Cancel gravity: F_gravity_comp = m * g (9.81 m/s²)
      // 2. Add proportional control to maintain target altitude (0.5m)
      auto pose = this->link_->WorldPose();
      double current_z = pose.Pos().Z();
      double target_z = 0.5;  // Target altitude (meters)
      double z_error = target_z - current_z;
      double vel_z = linear_vel.Z();

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

      // Publish current state to ROS2 (odometry feedback)
      this->PublishOdometry();
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

      // Construct odometry message
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = this->ros_node_->get_clock()->now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";

      // Position (meters)
      odom.pose.pose.position.x = pose.Pos().X();
      odom.pose.pose.position.y = pose.Pos().Y();
      odom.pose.pose.position.z = pose.Pos().Z();

      // Orientation (quaternion)
      odom.pose.pose.orientation.x = pose.Rot().X();
      odom.pose.pose.orientation.y = pose.Rot().Y();
      odom.pose.pose.orientation.z = pose.Rot().Z();
      odom.pose.pose.orientation.w = pose.Rot().W();

      // Linear velocity (m/s)
      odom.twist.twist.linear.x = linear_vel.X();
      odom.twist.twist.linear.y = linear_vel.Y();
      odom.twist.twist.linear.z = linear_vel.Z();

      // Angular velocity (rad/s)
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

    // Damping / stabilization parameters (tuned for 2D hover behavior)
    double linear_damping_ = 1.5;        ///< Linear velocity damping coefficient
    double angular_damping_ = 0.2;       ///< Angular velocity damping coefficient
    double attitude_stiffness_ = 4.0;    ///< Restoring torque for roll/pitch
    double yaw_damping_ = 0.1;           ///< Yaw damping to prevent spinning
    double altitude_kp_ = 4.0;           ///< Altitude proportional gain
    double altitude_kd_ = 4.0;           ///< Altitude velocity damping gain
  };

  // Register this plugin with Gazebo
  GZ_REGISTER_MODEL_PLUGIN(SimpleDronePlugin)
}  // namespace gazebo
