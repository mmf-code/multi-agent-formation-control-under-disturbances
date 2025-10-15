/*
 * Simple Drone Gazebo Plugin
 * Subscribes to: /agent_X/cmd_accel (geometry_msgs/Vector3)
 * Publishes to:  /agent_X/odom (nav_msgs/Odometry)
 * Applies force to drone based on commanded acceleration
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
  class SimpleDronePlugin : public ModelPlugin
  {
  public:
    SimpleDronePlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      // Store model pointer
      this->model_ = _model;
      this->link_ = this->model_->GetLink("base_link");

      if (!this->link_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("SimpleDronePlugin"),
                     "base_link not found in model!");
        return;
      }

      // Get drone mass
      this->mass_ = this->link_->GetInertial()->Mass();

      // Get namespace from SDF (e.g., "agent_0")
      if (_sdf->HasElement("namespace"))
      {
        this->namespace_ = _sdf->Get<std::string>("namespace");
      }
      else
      {
        this->namespace_ = this->model_->GetName();
      }

      // Initialize ROS2 node
      this->ros_node_ = gazebo_ros::Node::Get(_sdf);

      RCLCPP_INFO(this->ros_node_->get_logger(),
                  "SimpleDronePlugin loading for namespace: %s, mass: %.2f kg",
                  this->namespace_.c_str(), this->mass_);

      // Subscribe to cmd_accel
      std::string cmd_accel_topic = "/" + this->namespace_ + "/cmd_accel";
      this->cmd_accel_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
        cmd_accel_topic, 10,
        std::bind(&SimpleDronePlugin::OnCmdAccel, this, std::placeholders::_1));

      // Publisher for odometry
      std::string odom_topic = "/" + this->namespace_ + "/odom";
      this->odom_pub_ = this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, 10);

      // Connect to Gazebo update event
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SimpleDronePlugin::OnUpdate, this));

      RCLCPP_INFO(this->ros_node_->get_logger(),
                  "SimpleDronePlugin loaded successfully!");
    }

  private:
    void OnCmdAccel(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
      // Store commanded acceleration
      this->cmd_accel_x_ = msg->x;
      this->cmd_accel_y_ = msg->y;
      // Z locked for 2D simulation
      this->cmd_accel_z_ = 0.0;
    }

    void OnUpdate()
    {
      if (!this->link_)
        return;

      // Apply force = mass * acceleration (2D plane only)
      ignition::math::Vector3d force(
        this->mass_ * this->cmd_accel_x_,
        this->mass_ * this->cmd_accel_y_,
        0.0
      );

      // Full gravity compensation + Z position lock
      auto pose = this->link_->WorldPose();
      double current_z = pose.Pos().Z();
      double target_z = 0.5;  // Keep drone at 0.5m height
      double z_error = target_z - current_z;
      double z_force = this->mass_ * 9.81 + this->mass_ * z_error * 10.0;  // P control for Z

      force.Z(z_force);

      this->link_->AddForce(force);

      // Publish odometry
      this->PublishOdometry();
    }

    void PublishOdometry()
    {
      auto pose = this->link_->WorldPose();
      auto linear_vel = this->link_->WorldLinearVel();
      auto angular_vel = this->link_->WorldAngularVel();

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = this->ros_node_->get_clock()->now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";

      // Position
      odom.pose.pose.position.x = pose.Pos().X();
      odom.pose.pose.position.y = pose.Pos().Y();
      odom.pose.pose.position.z = pose.Pos().Z();

      // Orientation
      odom.pose.pose.orientation.x = pose.Rot().X();
      odom.pose.pose.orientation.y = pose.Rot().Y();
      odom.pose.pose.orientation.z = pose.Rot().Z();
      odom.pose.pose.orientation.w = pose.Rot().W();

      // Linear velocity
      odom.twist.twist.linear.x = linear_vel.X();
      odom.twist.twist.linear.y = linear_vel.Y();
      odom.twist.twist.linear.z = linear_vel.Z();

      // Angular velocity
      odom.twist.twist.angular.x = angular_vel.X();
      odom.twist.twist.angular.y = angular_vel.Y();
      odom.twist.twist.angular.z = angular_vel.Z();

      this->odom_pub_->publish(odom);
    }

    // Model and link pointers
    physics::ModelPtr model_;
    physics::LinkPtr link_;

    // ROS2
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cmd_accel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Gazebo update
    event::ConnectionPtr update_connection_;

    // Parameters
    std::string namespace_;
    double mass_;

    // Commanded acceleration
    double cmd_accel_x_ = 0.0;
    double cmd_accel_y_ = 0.0;
    double cmd_accel_z_ = 0.0;
  };

  GZ_REGISTER_MODEL_PLUGIN(SimpleDronePlugin)
}
