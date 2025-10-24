/**
 * @file metrics_publisher_node.cpp
 * @brief ROS2 node for computing and publishing real-time control performance metrics
 *
 * This node calculates key performance indicators (KPIs) for controller evaluation:
 *   - Tracking Error (instantaneous and RMSE)
 *   - Integral Absolute Error (IAE)
 *   - Integral Time-weighted Absolute Error (ITAE)
 *   - Settling Time Estimation
 *   - Overshoot Detection
 *
 * Subscribes:
 *   - /agent_X/odom (nav_msgs/Odometry): Current drone position
 *   - /agent_X/target_pose (geometry_msgs/PoseStamped): Desired position
 *
 * Publishes:
 *   - /agent_X/metrics (my_custom_interfaces_pkg/MetricsData): Real-time metrics
 *
 * @author Multi-Agent Formation Control Team
 * @date 2025-10-17
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <my_custom_interfaces_pkg/msg/metrics_data.hpp>

#include <cmath>
#include <memory>
#include <deque>
#include <functional>
#include <chrono>
#include <algorithm>

namespace agent_control_pkg
{

/**
 * @class MetricsPublisherNode
 * @brief Computes and publishes controller performance metrics in real-time
 */
class MetricsPublisherNode : public rclcpp::Node
{
public:
  MetricsPublisherNode()
  : Node("metrics_publisher_node")
  {
    // Declare parameters
    this->declare_parameter<double>("publish_rate_hz", 10.0);
    this->declare_parameter<double>("settling_threshold", 0.05);  // 5cm
    this->declare_parameter<double>("settling_time_window", 2.0);  // 2 seconds
    this->declare_parameter<int>("history_size", 1000);

    // Load parameters
    const double publish_rate = this->get_parameter("publish_rate_hz").as_double();
    settling_threshold_ = this->get_parameter("settling_threshold").as_double();
    settling_time_window_ = this->get_parameter("settling_time_window").as_double();
    const int history_size = this->get_parameter("history_size").as_int();

    error_history_.set_capacity(history_size);

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      rclcpp::SensorDataQoS(),
      std::bind(&MetricsPublisherNode::odomCallback, this, std::placeholders::_1)
    );

    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose",
      rclcpp::SensorDataQoS(),
      std::bind(&MetricsPublisherNode::targetCallback, this, std::placeholders::_1)
    );

    // Publisher (using proper MetricsData message type)
    metrics_pub_ = this->create_publisher<my_custom_interfaces_pkg::msg::MetricsData>("metrics", 10);

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      period,
      std::bind(&MetricsPublisherNode::computeAndPublishMetrics, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Metrics Publisher initialized: rate=%.1f Hz, settling_threshold=%.3f m",
      publish_rate, settling_threshold_
    );
  }

private:
  struct ErrorSample
  {
    rclcpp::Time timestamp;
    double error_x;
    double error_y;
    double error_z;
    double error_magnitude;
  };

  class CircularBuffer
  {
  public:
    void set_capacity(size_t cap) { capacity_ = cap; }

    void push_back(const ErrorSample& sample)
    {
      buffer_.push_back(sample);
      if (buffer_.size() > capacity_)
      {
        buffer_.pop_front();
      }
    }

    size_t size() const { return buffer_.size(); }
    bool empty() const { return buffer_.empty(); }
    const ErrorSample& operator[](size_t idx) const { return buffer_[idx]; }
    const ErrorSample& back() const { return buffer_.back(); }

  private:
    std::deque<ErrorSample> buffer_;
    size_t capacity_ = 1000;
  };

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_odom_ = msg;
    has_odom_ = true;
    updateMetrics();
  }

  void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_target_ = msg;
    has_target_ = true;

    // Detect target change (new setpoint)
    if (!prev_target_ ||
        std::abs(msg->pose.position.x - prev_target_->pose.position.x) > 0.01 ||
        std::abs(msg->pose.position.y - prev_target_->pose.position.y) > 0.01 ||
        std::abs(msg->pose.position.z - prev_target_->pose.position.z) > 0.01)
    {
      // Reset metrics on new target
      resetMetrics();
      prev_target_ = msg;
    }
  }

  void resetMetrics()
  {
    error_history_ = CircularBuffer();
    error_history_.set_capacity(this->get_parameter("history_size").as_int());

    iae_x_ = 0.0;
    iae_y_ = 0.0;
    itae_x_ = 0.0;
    itae_y_ = 0.0;
    max_overshoot_x_ = 0.0;
    max_overshoot_y_ = 0.0;
    settling_time_ = -1.0;
    start_time_ = this->now();

    RCLCPP_DEBUG(this->get_logger(), "Metrics reset - new target detected");
  }

  void updateMetrics()
  {
    if (!has_odom_ || !has_target_)
    {
      return;
    }

    const auto now = this->now();
    const double elapsed = (now - start_time_).seconds();

    // Compute tracking errors
    const double ex = current_odom_->pose.pose.position.x - current_target_->pose.position.x;
    const double ey = current_odom_->pose.pose.position.y - current_target_->pose.position.y;
    const double ez = current_odom_->pose.pose.position.z - current_target_->pose.position.z;
    const double error_mag = std::sqrt(ex*ex + ey*ey + ez*ez);

    // Store error sample
    ErrorSample sample;
    sample.timestamp = now;
    sample.error_x = ex;
    sample.error_y = ey;
    sample.error_z = ez;
    sample.error_magnitude = error_mag;
    error_history_.push_back(sample);

    // Compute dt for integration
    double dt = 0.01;  // Default
    if (error_history_.size() >= 2)
    {
      const auto& prev = error_history_[error_history_.size() - 2];
      dt = (now - prev.timestamp).seconds();
      dt = std::clamp(dt, 0.001, 0.1);  // Safety bounds
    }

    // Update integral metrics
    iae_x_ += std::abs(ex) * dt;
    iae_y_ += std::abs(ey) * dt;
    itae_x_ += elapsed * std::abs(ex) * dt;
    itae_y_ += elapsed * std::abs(ey) * dt;

    // Track overshoot (max error magnitude after initial approach)
    if (elapsed > 0.5)  // Ignore first 0.5s (initial transient)
    {
      max_overshoot_x_ = std::max(max_overshoot_x_, std::abs(ex));
      max_overshoot_y_ = std::max(max_overshoot_y_, std::abs(ey));
    }

    // Estimate settling time (last time error exceeded threshold)
    if (error_mag > settling_threshold_)
    {
      settling_time_ = elapsed;
    }
  }

  void computeAndPublishMetrics()
  {
    // Bail out during shutdown to avoid races on destruction
    if (!rclcpp::ok()) {
      return;
    }

    if (!has_odom_ || !has_target_ || error_history_.empty())
    {
      return;
    }

    // Compute RMSE over recent history
    const size_t n = error_history_.size();
    double sum_sq_x = 0.0;
    double sum_sq_y = 0.0;
    double sum_sq_z = 0.0;

    for (size_t i = 0; i < n; ++i)
    {
      const auto& sample = error_history_[i];
      sum_sq_x += sample.error_x * sample.error_x;
      sum_sq_y += sample.error_y * sample.error_y;
      sum_sq_z += sample.error_z * sample.error_z;
    }

    const double rmse_x = std::sqrt(sum_sq_x / n);
    const double rmse_y = std::sqrt(sum_sq_y / n);
    const double rmse_z = std::sqrt(sum_sq_z / n);
    const double rmse_total = std::sqrt((sum_sq_x + sum_sq_y + sum_sq_z) / n);

    // Current instantaneous error
    const auto& latest = error_history_.back();
    const double current_error = latest.error_magnitude;

    // Elapsed time
    const double elapsed = (this->now() - start_time_).seconds();

    // Check if settled (within threshold for settling_time_window_)
    bool is_settled = false;
    if (elapsed > settling_time_window_)
    {
      is_settled = true;
      const size_t window_samples = static_cast<size_t>(settling_time_window_ * 100);  // Assume 100Hz
      const size_t start_idx = n > window_samples ? n - window_samples : 0;

      for (size_t i = start_idx; i < n; ++i)
      {
        if (error_history_[i].error_magnitude > settling_threshold_)
        {
          is_settled = false;
          break;
        }
      }
    }

    // Publish metrics using proper MetricsData message
    my_custom_interfaces_pkg::msg::MetricsData msg;

    // Position errors
    msg.error_x = latest.error_x;
    msg.error_y = latest.error_y;
    msg.error_z = latest.error_z;
    msg.error_magnitude = current_error;

    // RMSE
    msg.rmse_x = rmse_x;
    msg.rmse_y = rmse_y;
    msg.rmse_z = rmse_z;
    msg.rmse_total = rmse_total;

    // Integral metrics
    msg.iae_x = iae_x_;
    msg.iae_y = iae_y_;
    msg.itae_x = itae_x_;
    msg.itae_y = itae_y_;

    // Response characteristics
    msg.settling_time = settling_time_;
    msg.is_settled = is_settled;
    msg.max_overshoot_x = max_overshoot_x_;
    msg.max_overshoot_y = max_overshoot_y_;

    metrics_pub_->publish(msg);

    // Log periodic summary
    static int log_counter = 0;
    if (++log_counter % 50 == 0)  // Every 5 seconds at 10Hz
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Metrics: error=%.3fm, RMSE=%.3fm, IAE=(%.2f,%.2f), settling_time=%.2fs, settled=%s",
        current_error, rmse_total, iae_x_, iae_y_, settling_time_,
        is_settled ? "YES" : "NO"
      );
    }
  }

  // ROS2 interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<my_custom_interfaces_pkg::msg::MetricsData>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_target_;
  geometry_msgs::msg::PoseStamped::SharedPtr prev_target_;
  bool has_odom_ = false;
  bool has_target_ = false;

  // Metrics
  CircularBuffer error_history_;
  double iae_x_ = 0.0;
  double iae_y_ = 0.0;
  double itae_x_ = 0.0;
  double itae_y_ = 0.0;
  double max_overshoot_x_ = 0.0;
  double max_overshoot_y_ = 0.0;
  double settling_time_ = -1.0;
  rclcpp::Time start_time_;

  // Parameters
  double settling_threshold_;
  double settling_time_window_;
};

}  // namespace agent_control_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agent_control_pkg::MetricsPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
