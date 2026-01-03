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
#include <geometry_msgs/msg/vector3.hpp>
#include <my_custom_interfaces_pkg/msg/metrics_data.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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
    // Settled detection (tunable from launch)
    this->declare_parameter<double>("settled_pos_threshold", 0.10);   // 10 cm
    this->declare_parameter<double>("settled_time_window", 1.0);      // 1 second
    // History / metrics configuration
    this->declare_parameter<int>("history_size", 1000);
    this->declare_parameter<double>("metrics_window_sec", 0.0);       // 0 → disabled
    this->declare_parameter<bool>("enable_group_metrics", false);

    // Load parameters
    const double publish_rate = this->get_parameter("publish_rate_hz").as_double();
    settled_pos_threshold_ = this->get_parameter("settled_pos_threshold").as_double();
    settled_time_window_ = this->get_parameter("settled_time_window").as_double();
    const int history_size = this->get_parameter("history_size").as_int();
    metrics_window_sec_ = this->get_parameter("metrics_window_sec").as_double();
    enable_group_metrics_ = this->get_parameter("enable_group_metrics").as_bool();

    error_history_.set_capacity(history_size);

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      rclcpp::SensorDataQoS(),
      std::bind(&MetricsPublisherNode::odomCallback, this, std::placeholders::_1)
    );

    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "target_pose",
      10,
      std::bind(&MetricsPublisherNode::targetCallback, this, std::placeholders::_1)
    );

    // Subscribe to wind velocity for wind disturbance metrics (global topic)
    wind_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/wind/velocity",
      10,
      std::bind(&MetricsPublisherNode::windCallback, this, std::placeholders::_1)
    );

    // Publisher (using proper MetricsData message type)
    metrics_pub_ = this->create_publisher<my_custom_interfaces_pkg::msg::MetricsData>("metrics", 10);

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      period,
      std::bind(&MetricsPublisherNode::computeAndPublishMetrics, this)
    );

    // Periodic status reporting to help diagnose missing inputs
    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MetricsPublisherNode::statusReport, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Metrics Publisher initialized: rate=%.1f Hz, settled_pos_threshold=%.3f m, settled_time_window=%.2f s",
      publish_rate, settled_pos_threshold_, settled_time_window_
    );

    // Optional group-level metrics publisher (per formation group)
    if (enable_group_metrics_ && metrics_window_sec_ > 0.0) {
      // Expect namespaces like "/agent_0", "/agent_6", etc.
      const std::string ns = this->get_namespace();
      int agent_index = -1;
      auto pos = ns.rfind("agent_");
      if (pos != std::string::npos) {
        std::string index_str = ns.substr(pos + 6);
        try {
          agent_index = std::stoi(index_str);
        } catch (const std::exception &) {
          agent_index = -1;
        }
      }

      if (agent_index >= 0) {
        const int group_index = agent_index / 3;  // 3 agents per group in this demo
        // Only create one publisher per group (representative = first agent)
        is_group_representative_ = (agent_index % 3 == 0);

        if (is_group_representative_) {
          const std::string topic = "/metrics/group_" + std::to_string(group_index);
          group_metrics_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
          RCLCPP_INFO(
            this->get_logger(),
            "Group metrics enabled for namespace '%s' (agent_%d, group_%d) on topic '%s' (window=%.1fs)",
            ns.c_str(), agent_index, group_index, topic.c_str(), metrics_window_sec_);
        }
      }
    }
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

    // First target received - initialize mission metrics
    if (!first_target_received_)
    {
      resetMissionMetrics();
      first_target_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Mission started - mission-wide metrics tracking enabled");
    }

    // Detect target change (new setpoint/waypoint)
    if (!prev_target_ ||
        std::abs(msg->pose.position.x - prev_target_->pose.position.x) > 0.01 ||
        std::abs(msg->pose.position.y - prev_target_->pose.position.y) > 0.01 ||
        std::abs(msg->pose.position.z - prev_target_->pose.position.z) > 0.01)
    {
      // Reset per-waypoint metrics only (mission-wide metrics continue accumulating)
      resetWaypointMetrics();
      waypoint_count_++;
      prev_target_ = msg;

      RCLCPP_INFO(
        this->get_logger(),
        "New waypoint #%d: (%.2f, %.2f, %.2f) - per-waypoint metrics reset",
        waypoint_count_, msg->pose.position.x, msg->pose.position.y, msg->pose.position.z
      );
    }
  }

  void windCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    wind_vx_ = msg->x;
    wind_vy_ = msg->y;
    wind_vz_ = msg->z;
  }

  /**
   * @brief Reset per-waypoint metrics (called at each waypoint transition)
   */
  void resetWaypointMetrics()
  {
    error_history_ = CircularBuffer();
    error_history_.set_capacity(this->get_parameter("history_size").as_int());

    // Per-waypoint metrics
    iae_x_ = 0.0;
    iae_y_ = 0.0;
    itae_x_ = 0.0;
    itae_y_ = 0.0;
    max_overshoot_x_ = 0.0;
    max_overshoot_y_ = 0.0;
    settling_time_ = -1.0;
    start_time_ = this->now();

    // Wind exposure time (per-waypoint tracking)
    wind_exposure_time_ = 0.0;

    RCLCPP_DEBUG(this->get_logger(), "Per-waypoint metrics reset");
  }

  /**
   * @brief Reset mission-wide cumulative metrics (called ONLY at mission start)
   */
  void resetMissionMetrics()
  {
    mission_sse_ = 0.0;
    mission_itae_x_ = 0.0;
    mission_itae_y_ = 0.0;
    mission_sample_count_ = 0;
    waypoint_count_ = 0;
    mission_start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Mission-wide metrics initialized");
  }

  void updateMetrics()
  {
    if (!has_odom_ || !has_target_)
    {
      return;
    }

    const auto now = this->now();
    const double elapsed = (now - start_time_).seconds();
    const double mission_elapsed = (now - mission_start_time_).seconds();

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

    // Update PER-WAYPOINT integral metrics
    iae_x_ += std::abs(ex) * dt;
    iae_y_ += std::abs(ey) * dt;
    itae_x_ += elapsed * std::abs(ex) * dt;
    itae_y_ += elapsed * std::abs(ey) * dt;

    // Update MISSION-WIDE cumulative metrics (never reset)
    if (first_target_received_)
    {
      mission_sse_ += error_mag * error_mag;  // Sum of squared errors for true RMSE
      mission_itae_x_ += mission_elapsed * std::abs(ex) * dt;
      mission_itae_y_ += mission_elapsed * std::abs(ey) * dt;
      mission_sample_count_++;
    }

    // Track overshoot (max error magnitude after initial approach)
    if (elapsed > 0.5)  // Ignore first 0.5s (initial transient)
    {
      max_overshoot_x_ = std::max(max_overshoot_x_, std::abs(ex));
      max_overshoot_y_ = std::max(max_overshoot_y_, std::abs(ey));
    }

    // Estimate settling time (last time error exceeded threshold)
    if (error_mag > settled_pos_threshold_)
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

    // Check if settled (within threshold for settled_time_window_)
    bool is_settled = false;
    if (elapsed > settled_time_window_)
    {
      is_settled = true;
      const size_t window_samples = static_cast<size_t>(settled_time_window_ * 100);  // Assume 100Hz
      const size_t start_idx = n > window_samples ? n - window_samples : 0;

      for (size_t i = start_idx; i < n; ++i)
      {
        if (error_history_[i].error_magnitude > settled_pos_threshold_)
        {
          is_settled = false;
          break;
        }
      }
    }

    // Publish metrics using proper MetricsData message
    my_custom_interfaces_pkg::msg::MetricsData msg;

    // Current and target positions (for step response analysis)
    msg.current_x = current_odom_->pose.pose.position.x;
    msg.current_y = current_odom_->pose.pose.position.y;
    msg.current_z = current_odom_->pose.pose.position.z;
    msg.target_x = current_target_->pose.position.x;
    msg.target_y = current_target_->pose.position.y;
    msg.target_z = current_target_->pose.position.z;

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

    // Mission-wide cumulative metrics
    if (first_target_received_ && mission_sample_count_ > 0)
    {
      msg.mission_rmse = std::sqrt(mission_sse_ / mission_sample_count_);  // True RMSE formula
      msg.mission_itae_x = mission_itae_x_;
      msg.mission_itae_y = mission_itae_y_;
      msg.mission_elapsed_time = (this->now() - mission_start_time_).seconds();
      msg.waypoint_count = waypoint_count_;
    }
    else
    {
      // No mission data yet
      msg.mission_rmse = 0.0;
      msg.mission_itae_x = 0.0;
      msg.mission_itae_y = 0.0;
      msg.mission_elapsed_time = 0.0;
      msg.waypoint_count = 0;
    }

    // Wind disturbance metrics (Phase 1 - Defense Industry Testing)
    msg.wind_velocity_x = wind_vx_;
    msg.wind_velocity_y = wind_vy_;
    msg.wind_velocity_z = wind_vz_;
    msg.wind_magnitude = std::sqrt(wind_vx_*wind_vx_ + wind_vy_*wind_vy_ + wind_vz_*wind_vz_);

    // Disturbance Rejection Ratio (DRR): |error| / |wind|
    const double error_mag = std::sqrt(latest.error_x*latest.error_x + latest.error_y*latest.error_y);
    msg.disturbance_rejection_ratio = (msg.wind_magnitude > 0.1)
        ? (error_mag / msg.wind_magnitude)
        : 0.0;

    // Wind-Normalized RMSE
    msg.wind_normalized_rmse = (msg.wind_magnitude > 0.1)
        ? (rmse_total / msg.wind_magnitude)
        : rmse_total;

    // Accumulate wind exposure time
    if (msg.wind_magnitude > WIND_THRESHOLD) {
      if (!error_history_.empty() && error_history_.size() >= 2) {
        const auto& prev = error_history_[error_history_.size() - 2];
        const auto& curr = error_history_[error_history_.size() - 1];
        const double dt = (curr.timestamp - prev.timestamp).seconds();
        wind_exposure_time_ += dt;
      }
    }
    msg.wind_exposure_time = wind_exposure_time_;

    metrics_pub_->publish(msg);

    // Optionally publish group-level metrics (windowed RMSE + IAE) for comparison
    if (group_metrics_pub_ && is_group_representative_ && metrics_window_sec_ > 0.0) {
      const auto now_time = this->now();
      const auto window_start = now_time - rclcpp::Duration::from_seconds(metrics_window_sec_);
      const double window_duration = (now_time - window_start).seconds();

      if (!error_history_.empty() && window_duration > 0.0) {
        double iae_window = 0.0;
        double sum_sq_window = 0.0;

        // Integrate |error| and error^2 over the window using sample-to-sample dt.
        for (size_t i = 1; i < n; ++i) {
          const auto &prev = error_history_[i - 1];
          const auto &curr = error_history_[i];

          if (curr.timestamp <= window_start) {
            continue;
          }

          rclcpp::Time seg_start = prev.timestamp;
          if (seg_start < window_start) {
            seg_start = window_start;
          }
          const double dt = (curr.timestamp - seg_start).seconds();
          if (dt <= 0.0) {
            continue;
          }

          const double e = curr.error_magnitude;
          const double abs_e = std::abs(e);
          iae_window += abs_e * dt;
          sum_sq_window += e * e * dt;
        }

        // Also integrate from last sample up to "now" using its latest error
        const auto &last_sample = error_history_.back();
        if (last_sample.timestamp > window_start) {
          double dt_last = (now_time - last_sample.timestamp).seconds();
          if (dt_last > 0.0) {
            const double e_last = last_sample.error_magnitude;
            const double abs_e_last = std::abs(e_last);
            iae_window += abs_e_last * dt_last;
            sum_sq_window += e_last * e_last * dt_last;
          }
        }

        double rmse_window = 0.0;
        if (sum_sq_window > 0.0 && window_duration > 0.0) {
          rmse_window = std::sqrt(sum_sq_window / window_duration);
        }

        std_msgs::msg::Float64MultiArray group_msg;
        group_msg.data = {
          rmse_window,
          iae_window,
          window_duration
        };
        group_metrics_pub_->publish(group_msg);
      }
    }

    // Log periodic summary
    static int log_counter = 0;
    if (++log_counter % 50 == 0)  // Every 5 seconds at 10Hz
    {
      if (first_target_received_ && mission_sample_count_ > 0)
      {
        const double mission_rmse = std::sqrt(mission_sse_ / mission_sample_count_);
        RCLCPP_INFO(
          this->get_logger(),
          "Metrics | WP #%d: error=%.3fm, RMSE=%.3fm, IAE=(%.2f,%.2f), settling=%.2fs | "
          "Mission: RMSE=%.3fm, ITAE=(%.2f,%.2f), elapsed=%.1fs",
          waypoint_count_, current_error, rmse_total, iae_x_, iae_y_, settling_time_,
          mission_rmse, mission_itae_x_, mission_itae_y_, (this->now() - mission_start_time_).seconds()
        );
      }
      else
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Metrics: error=%.3fm, RMSE=%.3fm, IAE=(%.2f,%.2f), settling_time=%.2fs, settled=%s",
          current_error, rmse_total, iae_x_, iae_y_, settling_time_,
          is_settled ? "YES" : "NO"
        );
      }
    }
  }

  void statusReport()
  {
    // Report if upstream feeds are missing so users can spot why metrics are empty
    if (!has_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "No odom received yet on '%s/odom'", this->get_namespace());
    }
    if (!has_target_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "No target_pose received yet on '%s/target_pose'", this->get_namespace());
    }
  }

  // ROS2 interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_sub_;
  rclcpp::Publisher<my_custom_interfaces_pkg::msg::MetricsData>::SharedPtr metrics_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr group_metrics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // State
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_target_;
  geometry_msgs::msg::PoseStamped::SharedPtr prev_target_;
  bool has_odom_ = false;
  bool has_target_ = false;

  // Metrics - Per-waypoint (reset at each waypoint)
  CircularBuffer error_history_;
  double iae_x_ = 0.0;
  double iae_y_ = 0.0;
  double itae_x_ = 0.0;
  double itae_y_ = 0.0;
  double max_overshoot_x_ = 0.0;
  double max_overshoot_y_ = 0.0;
  double settling_time_ = -1.0;
  rclcpp::Time start_time_;  // Waypoint start time

  // Metrics - Mission-wide cumulative (NEVER reset, except at mission start)
  double mission_sse_ = 0.0;  // Sum of Squared Errors for true RMSE
  double mission_itae_x_ = 0.0;
  double mission_itae_y_ = 0.0;
  int mission_sample_count_ = 0;
  int waypoint_count_ = 0;
  rclcpp::Time mission_start_time_;
  bool first_target_received_ = false;

  // Wind disturbance tracking (Phase 1 - Defense Industry Testing)
  double wind_vx_ = 0.0;
  double wind_vy_ = 0.0;
  double wind_vz_ = 0.0;
  double wind_exposure_time_ = 0.0;
  static constexpr double WIND_THRESHOLD = 1.0;  // [m/s]

  // Parameters
  double settled_pos_threshold_{0.10};
  double settled_time_window_{1.0};
  double metrics_window_sec_{0.0};
  bool enable_group_metrics_{false};
  bool is_group_representative_{false};
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
