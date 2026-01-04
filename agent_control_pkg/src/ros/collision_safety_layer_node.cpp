/**
 * @file collision_safety_layer_node.cpp
 * @brief Implementation of Collision Safety Layer for Multi-Agent Formation Control
 *
 * This node provides APF-based collision avoidance as a safety layer between
 * formation coordinators and agent controllers. It modifies target positions
 * to reduce collision risk while preserving the original controller comparison
 * methodology.
 *
 * IMPORTANT: This is a SOFT CONSTRAINT approach. It does NOT guarantee
 * collision-free operation. For hard constraints, use CBF or RVO methods.
 *
 * @author Multi-Agent Formation Control Research
 * @date 2026-01-04
 */

#include "agent_control_pkg/collision_safety_layer.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <sstream>

namespace agent_control_pkg
{

CollisionSafetyLayer::CollisionSafetyLayer(const rclcpp::NodeOptions& options)
  : rclcpp::Node("collision_safety_layer", options)
{
  declareParameters();
  loadParameters();

  // Initialize agent states
  agent_states_.resize(num_agents_);

  // Log configuration
  RCLCPP_INFO(get_logger(),
    "Collision Safety Layer initialized: mode=%s, agents=%zu, "
    "safety_dist=%.2fm, influence_dist=%.2fm, k_rep=%.2f, max_dev=%.2fm",
    modeToString(mode_).c_str(), num_agents_,
    safety_distance_, influence_distance_, k_repulsion_, max_target_deviation_);

  if (mode_ == SafetyMode::DISABLED) {
    RCLCPP_INFO(get_logger(),
      "Mode is DISABLED - safety layer will not publish safe_target_pose. "
      "Use remapping only when mode is PASS_THROUGH or AVOIDANCE.");
    return;  // Don't set up subscribers/publishers in DISABLED mode
  }

  setupSubscribersAndPublishers();

  // Create timer for periodic safe target publishing
  const double period_sec = 1.0 / std::max(publish_rate_hz_, 1.0);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(period_sec),
    std::bind(&CollisionSafetyLayer::timerCallback, this));

  last_metrics_time_ = now();

  RCLCPP_INFO(get_logger(),
    "Safety layer active at %.1f Hz. Subscribing to %zu agents.",
    publish_rate_hz_, num_agents_);
}

void CollisionSafetyLayer::declareParameters()
{
  declare_parameter<std::string>("mode", "disabled");
  declare_parameter<int>("num_agents", 12);
  declare_parameter<double>("safety_distance", 0.8);
  declare_parameter<double>("influence_distance", 2.0);
  declare_parameter<double>("k_repulsion", 3.0);
  declare_parameter<double>("max_target_deviation", 0.5);
  declare_parameter<double>("publish_rate_hz", 20.0);
  declare_parameter<bool>("publish_metrics", true);
}

void CollisionSafetyLayer::loadParameters()
{
  const std::string mode_str = get_parameter("mode").as_string();
  mode_ = stringToMode(mode_str);

  num_agents_ = static_cast<size_t>(get_parameter("num_agents").as_int());
  safety_distance_ = get_parameter("safety_distance").as_double();
  influence_distance_ = get_parameter("influence_distance").as_double();
  k_repulsion_ = get_parameter("k_repulsion").as_double();
  max_target_deviation_ = get_parameter("max_target_deviation").as_double();
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

  // Store safety threshold in metrics
  metrics_.safety_threshold = safety_distance_;

  // Validate parameters
  if (influence_distance_ <= safety_distance_) {
    RCLCPP_WARN(get_logger(),
      "influence_distance (%.2f) should be > safety_distance (%.2f). Adjusting.",
      influence_distance_, safety_distance_);
    influence_distance_ = safety_distance_ * 2.5;
  }
}

SafetyMode CollisionSafetyLayer::stringToMode(const std::string& mode_str) const
{
  std::string lower = mode_str;
  std::transform(lower.begin(), lower.end(), lower.begin(),
    [](unsigned char c) { return std::tolower(c); });

  if (lower == "disabled" || lower == "off" || lower == "none") {
    return SafetyMode::DISABLED;
  } else if (lower == "pass_through" || lower == "passthrough" || lower == "pass-through") {
    return SafetyMode::PASS_THROUGH;
  } else if (lower == "avoidance" || lower == "apf" || lower == "on") {
    return SafetyMode::AVOIDANCE;
  }

  RCLCPP_WARN(get_logger(), "Unknown mode '%s', defaulting to DISABLED", mode_str.c_str());
  return SafetyMode::DISABLED;
}

std::string CollisionSafetyLayer::modeToString(SafetyMode mode) const
{
  switch (mode) {
    case SafetyMode::DISABLED: return "DISABLED";
    case SafetyMode::PASS_THROUGH: return "PASS_THROUGH";
    case SafetyMode::AVOIDANCE: return "AVOIDANCE";
    default: return "UNKNOWN";
  }
}

void CollisionSafetyLayer::setupSubscribersAndPublishers()
{
  odom_subs_.reserve(num_agents_);
  target_subs_.reserve(num_agents_);
  safe_target_pubs_.reserve(num_agents_);

  for (size_t i = 0; i < num_agents_; ++i) {
    const std::string agent_ns = "/agent_" + std::to_string(i);

    // Subscribe to odometry
    auto odom_cb = [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
      this->odomCallback(msg, i);
    };
    odom_subs_.push_back(
      create_subscription<nav_msgs::msg::Odometry>(
        agent_ns + "/odom", rclcpp::SensorDataQoS(), odom_cb));

    // Subscribe to original target pose
    auto target_cb = [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      this->targetCallback(msg, i);
    };
    target_subs_.push_back(
      create_subscription<geometry_msgs::msg::PoseStamped>(
        agent_ns + "/target_pose", 10, target_cb));

    // Publish safe target pose
    safe_target_pubs_.push_back(
      create_publisher<geometry_msgs::msg::PoseStamped>(
        agent_ns + "/safe_target_pose", 10));
  }

  // Metrics publisher
  if (get_parameter("publish_metrics").as_bool()) {
    metrics_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/collision_safety/metrics", 10);
  }

  RCLCPP_INFO(get_logger(),
    "Set up %zu odom subs, %zu target subs, %zu safe_target pubs",
    odom_subs_.size(), target_subs_.size(), safe_target_pubs_.size());
}

void CollisionSafetyLayer::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg, size_t agent_idx)
{
  if (agent_idx >= num_agents_) return;

  std::lock_guard<std::mutex> lock(state_mutex_);
  auto& state = agent_states_[agent_idx];

  state.position.x = msg->pose.pose.position.x;
  state.position.y = msg->pose.pose.position.y;
  state.last_odom_time = now();
  state.has_odom = true;
}

void CollisionSafetyLayer::targetCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg, size_t agent_idx)
{
  if (agent_idx >= num_agents_) return;

  std::lock_guard<std::mutex> lock(state_mutex_);
  auto& state = agent_states_[agent_idx];

  state.target.x = msg->pose.position.x;
  state.target.y = msg->pose.position.y;
  state.target_z = msg->pose.position.z;
  state.last_target_msg = *msg;
  state.last_target_time = now();
  state.has_target = true;
}

void CollisionSafetyLayer::timerCallback()
{
  updateMetrics();
  publishSafeTargets();

  if (metrics_pub_) {
    publishMetrics();
  }
}

Position2D CollisionSafetyLayer::computeRepulsion(size_t agent_idx) const
{
  // Must be called with state_mutex_ held
  Position2D total_repulsion(0.0, 0.0);
  const auto& my_state = agent_states_[agent_idx];

  if (!my_state.has_odom) {
    return total_repulsion;
  }

  for (size_t j = 0; j < num_agents_; ++j) {
    if (j == agent_idx) continue;

    const auto& other_state = agent_states_[j];
    if (!other_state.has_odom) continue;

    // Vector from other to me
    Position2D diff = my_state.position - other_state.position;
    double dist = diff.norm();

    if (dist < 1e-6) {
      // Agents at same position - apply random small repulsion
      total_repulsion.x += 0.1;
      total_repulsion.y += 0.1;
      continue;
    }

    if (dist < influence_distance_) {
      // APF repulsion formula:
      // F_rep = k_rep * (1/d - 1/d_inf)^2 * direction
      // This creates stronger repulsion as agents get closer
      double inv_d = 1.0 / dist;
      double inv_d_inf = 1.0 / influence_distance_;
      double magnitude = k_repulsion_ * std::pow(inv_d - inv_d_inf, 2);

      // Direction: away from other agent
      Position2D direction = diff.normalized();
      total_repulsion = total_repulsion + direction * magnitude;
    }
  }

  return total_repulsion;
}

Position2D CollisionSafetyLayer::clampMagnitude(const Position2D& vec, double max_mag) const
{
  double mag = vec.norm();
  if (mag <= max_mag || mag < 1e-9) {
    return vec;
  }
  return vec * (max_mag / mag);
}

Position2D CollisionSafetyLayer::computeSafeTarget(size_t agent_idx) const
{
  // Must be called with state_mutex_ held
  const auto& state = agent_states_[agent_idx];

  if (!state.has_target) {
    return state.position;  // Fallback to current position
  }

  if (mode_ == SafetyMode::PASS_THROUGH) {
    // No modification - just pass through original target
    return state.target;
  }

  // AVOIDANCE mode: Apply APF repulsion to target
  Position2D repulsion = computeRepulsion(agent_idx);

  // Clamp repulsion to max deviation
  repulsion = clampMagnitude(repulsion, max_target_deviation_);

  // Apply repulsion to original target
  return state.target + repulsion;
}

void CollisionSafetyLayer::updateMetrics()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Compute current minimum inter-agent distance
  double min_dist = std::numeric_limits<double>::max();
  double sum_dist = 0.0;
  int pair_count = 0;

  for (size_t i = 0; i < num_agents_; ++i) {
    if (!agent_states_[i].has_odom) continue;

    for (size_t j = i + 1; j < num_agents_; ++j) {
      if (!agent_states_[j].has_odom) continue;

      Position2D diff = agent_states_[i].position - agent_states_[j].position;
      double dist = diff.norm();

      min_dist = std::min(min_dist, dist);
      sum_dist += dist;
      pair_count++;
    }
  }

  if (pair_count > 0) {
    metrics_.min_inter_agent_distance = min_dist;
    metrics_.avg_inter_agent_distance = sum_dist / pair_count;

    // Update all-time minimum
    if (min_dist < metrics_.min_distance_ever) {
      metrics_.min_distance_ever = min_dist;
    }

    // Check for near-miss
    if (min_dist < safety_distance_) {
      metrics_.near_miss_count++;

      // Accumulate time below threshold
      if (metrics_initialized_) {
        double dt = (now() - last_metrics_time_).seconds();
        metrics_.time_below_threshold_sec += dt;
      }
    }
  }

  last_metrics_time_ = now();
  metrics_initialized_ = true;
}

void CollisionSafetyLayer::publishSafeTargets()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  for (size_t i = 0; i < num_agents_; ++i) {
    const auto& state = agent_states_[i];

    if (!state.has_target) {
      continue;  // No target received yet
    }

    // Compute safe target
    Position2D safe_pos = computeSafeTarget(i);

    // Create message based on original target
    geometry_msgs::msg::PoseStamped safe_msg = state.last_target_msg;
    safe_msg.header.stamp = now();  // Update timestamp
    safe_msg.pose.position.x = safe_pos.x;
    safe_msg.pose.position.y = safe_pos.y;
    // Preserve Z and orientation from original target

    safe_target_pubs_[i]->publish(safe_msg);
  }
}

void CollisionSafetyLayer::publishMetrics()
{
  std_msgs::msg::Float64MultiArray msg;

  // Layout: [min_dist, min_ever, near_miss_count, time_below, avg_dist, threshold, mode]
  msg.data.resize(7);
  msg.data[0] = metrics_.min_inter_agent_distance;
  msg.data[1] = metrics_.min_distance_ever;
  msg.data[2] = static_cast<double>(metrics_.near_miss_count);
  msg.data[3] = metrics_.time_below_threshold_sec;
  msg.data[4] = metrics_.avg_inter_agent_distance;
  msg.data[5] = metrics_.safety_threshold;
  msg.data[6] = static_cast<double>(mode_);

  metrics_pub_->publish(msg);
}

}  // namespace agent_control_pkg

// Main entry point
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agent_control_pkg::CollisionSafetyLayer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
