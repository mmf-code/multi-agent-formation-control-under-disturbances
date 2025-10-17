/**
 * @file path_visualizer_node.cpp
 * @brief ROS2 node for visualizing drone trajectory path in RViz2
 *
 * This node subscribes to odometry messages and publishes a Path message
 * for visualization in RViz2. It provides a visual trail of the drone's
 * movement history for presentation and analysis purposes.
 *
 * Subscribes:
 *   - /agent_X/odom (nav_msgs/Odometry): Drone state feedback
 *
 * Publishes:
 *   - /agent_X/path (nav_msgs/Path): Trajectory path for RViz2
 *
 * @author Multi-Agent Formation Control Team
 * @date 2025-10-17
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <deque>
#include <functional>
#include <chrono>
#include <cmath>
#include <string>

namespace agent_control_pkg
{

/**
 * @class PathVisualizerNode
 * @brief Generates and publishes trajectory path for RViz2 visualization
 *
 * This node maintains a history of drone positions and publishes them
 * as a Path message. The path length is configurable via ROS2 parameters.
 */
class PathVisualizerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor - initializes subscribers, publishers, and parameters
   */
  PathVisualizerNode()
  : Node("path_visualizer_node")
  {
    // Declare parameters
    this->declare_parameter<int>("max_path_length", 1000);
    this->declare_parameter<double>("publish_rate_hz", 10.0);
    this->declare_parameter<double>("min_distance_threshold", 0.01);
    this->declare_parameter<std::string>("frame_id", "odom");

    // Load parameters
    max_path_length_ = this->get_parameter("max_path_length").as_int();
    const double publish_rate = this->get_parameter("publish_rate_hz").as_double();
    min_distance_threshold_ = this->get_parameter("min_distance_threshold").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // Create subscriber for odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      rclcpp::SensorDataQoS(),
      std::bind(&PathVisualizerNode::odomCallback, this, std::placeholders::_1)
    );

    // Create publisher for path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

    // Create timer for periodic publishing
    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      period,
      std::bind(&PathVisualizerNode::publishPath, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Path Visualizer Node initialized: max_length=%d, rate=%.1f Hz",
      max_path_length_, publish_rate
    );
  }

private:
  /**
   * @brief Callback for odometry messages
   * @param msg Odometry message containing drone position
   *
   * Adds new positions to the path buffer, filtering out points that
   * are too close together to reduce noise and improve visualization.
   */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Extract position from odometry
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = frame_id_;
    pose.pose = msg->pose.pose;

    // Check if we should add this point (distance filtering)
    if (shouldAddPoint(pose))
    {
      path_buffer_.push_back(pose);

      // Maintain maximum path length (FIFO buffer)
      if (static_cast<int>(path_buffer_.size()) > max_path_length_)
      {
        path_buffer_.pop_front();
      }

      last_pose_ = pose;
    }
  }

  /**
   * @brief Check if a new point should be added to the path
   * @param pose New pose to evaluate
   * @return true if point should be added, false otherwise
   *
   * Filters points based on minimum distance threshold to prevent
   * cluttering the visualization with redundant points.
   */
  bool shouldAddPoint(const geometry_msgs::msg::PoseStamped& pose)
  {
    // Always add first point
    if (path_buffer_.empty())
    {
      return true;
    }

    // Calculate distance from last point
    const double dx = pose.pose.position.x - last_pose_.pose.position.x;
    const double dy = pose.pose.position.y - last_pose_.pose.position.y;
    const double dz = pose.pose.position.z - last_pose_.pose.position.z;
    const double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Add if moved beyond threshold
    return distance >= min_distance_threshold_;
  }

  /**
   * @brief Publish current path for RViz2 visualization
   *
   * This function is called periodically by the timer. It constructs
   * a Path message from the buffered poses and publishes it.
   */
  void publishPath()
  {
    if (path_buffer_.empty())
    {
      return;
    }

    // Construct path message
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = frame_id_;

    // Copy all poses from buffer
    path_msg.poses.reserve(path_buffer_.size());
    for (const auto& pose : path_buffer_)
    {
      path_msg.poses.push_back(pose);
    }

    // Publish path
    path_pub_->publish(path_msg);
  }

  // ROS2 interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Path buffer (FIFO queue)
  std::deque<geometry_msgs::msg::PoseStamped> path_buffer_;
  geometry_msgs::msg::PoseStamped last_pose_;

  // Parameters
  int max_path_length_;
  double min_distance_threshold_;
  std::string frame_id_;
};

}  // namespace agent_control_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agent_control_pkg::PathVisualizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
