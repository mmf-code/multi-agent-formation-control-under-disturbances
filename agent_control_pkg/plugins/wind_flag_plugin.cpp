/**
 * Wind Flag Plugin for Gazebo
 *
 * Rotates a wind flag/windsock model to point in the wind direction.
 * Subscribes to /wind/velocity ROS2 topic and calculates yaw angle.
 *
 * SDF Parameters:
 *   <joint_name>: Name of the revolute joint to control
 *   <wind_topic>: ROS2 topic for wind velocity (default: /wind/velocity)
 *   <update_rate>: Plugin update rate in Hz (default: 10)
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>
#include <mutex>

namespace gazebo
{
class WindFlagPlugin : public ModelPlugin
{
public:
  WindFlagPlugin() : ModelPlugin() {}

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->model_ = _model;
    this->world_ = _model->GetWorld();

    // Get joint name
    std::string joint_name = "flag_joint";
    if (_sdf->HasElement("joint_name")) {
      joint_name = _sdf->Get<std::string>("joint_name");
    }

    this->joint_ = _model->GetJoint(joint_name);
    if (!this->joint_) {
      gzerr << "WindFlagPlugin: Joint '" << joint_name << "' not found!\n";
      return;
    }

    // Get wind topic
    std::string wind_topic = "/wind/velocity";
    if (_sdf->HasElement("wind_topic")) {
      wind_topic = _sdf->Get<std::string>("wind_topic");
    }

    // Get update rate
    double update_rate = 10.0;
    if (_sdf->HasElement("update_rate")) {
      update_rate = _sdf->Get<double>("update_rate");
    }
    this->update_period_ = 1.0 / update_rate;

    // Initialize ROS2 node
    this->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Subscribe to wind velocity
    this->wind_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
      wind_topic, 10,
      std::bind(&WindFlagPlugin::OnWindVelocity, this, std::placeholders::_1));

    // Connect to world update event
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WindFlagPlugin::OnUpdate, this));

    gzmsg << "WindFlagPlugin loaded for model: " << _model->GetName()
          << ", joint: " << joint_name
          << ", topic: " << wind_topic << "\n";
  }

  void OnWindVelocity(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(this->wind_mutex_);
    this->wind_x_ = msg->x;
    this->wind_y_ = msg->y;
    this->has_wind_data_ = true;
  }

  void OnUpdate()
  {
    // Rate limiting
    common::Time current_time = this->world_->SimTime();
    double dt = (current_time - this->last_update_time_).Double();
    if (dt < this->update_period_) {
      return;
    }
    this->last_update_time_ = current_time;

    if (!this->joint_) {
      return;
    }

    double target_yaw = 0.0;

    {
      std::lock_guard<std::mutex> lock(this->wind_mutex_);
      if (this->has_wind_data_) {
        // Calculate wind direction (yaw angle)
        // Wind flag points in direction wind is blowing TO (opposite of from)
        double wind_speed = std::sqrt(this->wind_x_ * this->wind_x_ +
                                       this->wind_y_ * this->wind_y_);
        if (wind_speed > 0.1) {  // Only update if significant wind
          target_yaw = std::atan2(this->wind_y_, this->wind_x_);
        }
      }
    }

    // Smooth transition to target yaw
    double current_yaw = this->joint_->Position(0);
    double yaw_error = target_yaw - current_yaw;

    // Normalize angle to [-pi, pi]
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    // Apply proportional control with damping
    double kp = 2.0;  // Proportional gain
    double torque = kp * yaw_error;

    // Clamp torque
    torque = std::max(-1.0, std::min(1.0, torque));

    this->joint_->SetForce(0, torque);
  }

private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr joint_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_sub_;
  event::ConnectionPtr update_connection_;

  std::mutex wind_mutex_;
  double wind_x_ = 0.0;
  double wind_y_ = 0.0;
  bool has_wind_data_ = false;

  double update_period_ = 0.1;
  common::Time last_update_time_;
};

GZ_REGISTER_MODEL_PLUGIN(WindFlagPlugin)
}  // namespace gazebo
