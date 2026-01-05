/**
 * @file collision_safety_layer.hpp
 * @brief Collision Safety Layer for Multi-Agent Formation Control
 *
 * Provides collision avoidance through Artificial Potential Field (APF) based
 * target modification. This is a SOFT CONSTRAINT approach - it reduces collision
 * risk but does NOT guarantee collision-free operation.
 *
 * For hard constraints, consider CBF (Control Barrier Functions) or RVO
 * (Reciprocal Velocity Obstacles) approaches.
 *
 * Three operating modes:
 * - DISABLED: Bypass layer completely (for baseline comparison)
 * - PASS_THROUGH: Republish targets without modification (isolate republish effect)
 * - AVOIDANCE: Apply APF-based target modification
 *
 * @author Multi-Agent Formation Control Research
 * @date 2026-01-04
 */

#ifndef AGENT_CONTROL_PKG__COLLISION_SAFETY_LAYER_HPP_
#define AGENT_CONTROL_PKG__COLLISION_SAFETY_LAYER_HPP_

#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "my_custom_interfaces_pkg/msg/neighbor_info.hpp"
#include "my_custom_interfaces_pkg/msg/neighbor_position.hpp"

namespace agent_control_pkg
{

/**
 * @brief Safety layer operating mode
 */
enum class SafetyMode
{
  DISABLED,     ///< Layer bypassed, no safe_target published
  PASS_THROUGH, ///< Republish target without modification (isolate republish effect)
  AVOIDANCE     ///< APF-based collision avoidance active
};

/**
 * @brief 2D position structure
 */
struct Position2D
{
  double x{0.0};
  double y{0.0};

  Position2D() = default;
  Position2D(double x_, double y_) : x(x_), y(y_) {}

  Position2D operator+(const Position2D& other) const {
    return Position2D(x + other.x, y + other.y);
  }

  Position2D operator-(const Position2D& other) const {
    return Position2D(x - other.x, y - other.y);
  }

  Position2D operator*(double scalar) const {
    return Position2D(x * scalar, y * scalar);
  }

  double norm() const {
    return std::sqrt(x * x + y * y);
  }

  Position2D normalized() const {
    double n = norm();
    if (n < 1e-9) return Position2D(0.0, 0.0);
    return Position2D(x / n, y / n);
  }
};

/**
 * @brief Per-agent state tracking
 */
struct AgentState
{
  Position2D position;                                    ///< Current position from odometry
  Position2D velocity;                                    ///< Current velocity from odometry
  double position_z{0.0};                                 ///< Z component of position
  Position2D target;                                      ///< Original target from formation coordinator
  double target_z{1.0};                                   ///< Z component of target (preserved)
  geometry_msgs::msg::PoseStamped last_target_msg;        ///< Full target message for republishing
  rclcpp::Time last_odom_time;                            ///< Timestamp of last odometry update
  rclcpp::Time last_target_time;                          ///< Timestamp of last target update
  bool has_odom{false};                                   ///< Whether odometry has been received
  bool has_target{false};                                 ///< Whether target has been received
};

/**
 * @brief Safety metrics for monitoring
 */
struct SafetyMetrics
{
  double min_inter_agent_distance{std::numeric_limits<double>::max()};  ///< Current minimum distance
  double min_distance_ever{std::numeric_limits<double>::max()};         ///< All-time minimum
  uint32_t near_miss_count{0};                                          ///< Count of d < threshold events
  double time_below_threshold_sec{0.0};                                 ///< Cumulative time below threshold
  double avg_inter_agent_distance{0.0};                                 ///< Average distance
  double safety_threshold{0.8};                                         ///< Threshold for near-miss
};

/**
 * @brief Collision Safety Layer Node
 *
 * Subscribes to all agent odometry and target poses, computes collision-aware
 * safe targets using APF, and publishes modified targets.
 */
class CollisionSafetyLayer : public rclcpp::Node
{
public:
  explicit CollisionSafetyLayer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // === Parameter Management ===
  void declareParameters();
  void loadParameters();
  SafetyMode stringToMode(const std::string& mode_str) const;
  std::string modeToString(SafetyMode mode) const;

  // === Subscription Setup ===
  void setupSubscribersAndPublishers();

  // === Callbacks ===
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, size_t agent_idx);
  void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, size_t agent_idx);
  void timerCallback();

  // === APF Computation ===
  /**
   * @brief Compute APF repulsion force for an agent
   * @param agent_idx Index of the agent
   * @return Repulsion vector (2D)
   *
   * APF Formula:
   * F_rep = k_rep * (1/d - 1/d_influence)^2 * (p_i - p_j) / ||p_i - p_j||
   * Applied only when d < d_influence
   */
  Position2D computeRepulsion(size_t agent_idx) const;

  /**
   * @brief Compute safe target for an agent
   * @param agent_idx Index of the agent
   * @return Modified target position
   */
  Position2D computeSafeTarget(size_t agent_idx) const;

  /**
   * @brief Clamp a vector to maximum magnitude
   */
  Position2D clampMagnitude(const Position2D& vec, double max_mag) const;

  // === Metrics ===
  void updateMetrics();
  void publishMetrics();
  void publishSafeTargets();

  // === Plan S: Perception Hub ===
  /**
   * @brief Compute and publish neighbor info for all agents
   *
   * For each agent, finds all neighbors within perception_radius_,
   * computes relative distances and bearings, and publishes NeighborInfo.
   */
  void publishNeighborInfo();

  /**
   * @brief Build NeighborInfo message for a specific agent
   * @param agent_idx Index of the agent
   * @return NeighborInfo message with all neighbors within perception radius
   */
  my_custom_interfaces_pkg::msg::NeighborInfo buildNeighborInfo(size_t agent_idx) const;

  // === Member Variables ===

  // Mode
  SafetyMode mode_{SafetyMode::DISABLED};

  // Parameters
  size_t num_agents_{12};
  double safety_distance_{0.8};       ///< [m] Threshold for near-miss detection
  double influence_distance_{2.0};    ///< [m] APF influence radius
  double k_repulsion_{3.0};           ///< APF repulsion gain
  double max_target_deviation_{0.5};  ///< [m] Max modification to original target
  double publish_rate_hz_{20.0};      ///< Safe target publish rate

  // Plan S: Perception Hub parameters
  bool perception_hub_enable_{true};  ///< Enable neighbor info publishing
  double perception_radius_{10.0};    ///< [m] Max distance for neighbor inclusion

  // State tracking
  std::vector<AgentState> agent_states_;
  mutable std::mutex state_mutex_;

  // Metrics
  SafetyMetrics metrics_;
  rclcpp::Time last_metrics_time_;
  bool metrics_initialized_{false};

  // ROS interfaces
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> target_subs_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> safe_target_pubs_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Plan S: Perception Hub publishers
  std::vector<rclcpp::Publisher<my_custom_interfaces_pkg::msg::NeighborInfo>::SharedPtr> neighbor_info_pubs_;
};

}  // namespace agent_control_pkg

#endif  // AGENT_CONTROL_PKG__COLLISION_SAFETY_LAYER_HPP_
