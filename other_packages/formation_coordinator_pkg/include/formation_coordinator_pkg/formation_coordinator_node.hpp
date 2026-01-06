#ifndef FORMATION_COORDINATOR_PKG__FORMATION_COORDINATOR_NODE_HPP_
#define FORMATION_COORDINATOR_PKG__FORMATION_COORDINATOR_NODE_HPP_

#include <array>
#include <string>
#include <vector>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "my_custom_interfaces_pkg/msg/formation_state.hpp"
#include "my_custom_interfaces_pkg/srv/update_formation.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace formation_coordinator_pkg
{

// Formation shape enumeration for 4 supported formations
enum class FormationShape
{
  TRIANGLE,
  LINE,
  SQUARE,    // L-shape for 3 drones (incomplete square)
  V_SHAPE
};

// Trajectory mode enumeration for center motion
enum class TrajectoryMode
{
  WAYPOINTS,    // Linear interpolation between waypoints (default)
  CIRCULAR,     // Circular orbit around initial center
  LEMNISCATE,   // Figure-8 (Bernoulli lemniscate)
  SPIRAL        // Expanding/contracting spiral
};

class FormationCoordinatorNode : public rclcpp::Node
{
public:
  explicit FormationCoordinatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct AgentPublisher
  {
    std::string id;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
  };

  void declareParameters();
  void loadParameters();
  void rebuildPublishers();
  std::vector<std::array<double, 2>> computeOffsets(std::size_t count) const;
  std::vector<std::array<double, 2>> generateFormationPositions(FormationShape shape, std::size_t count) const;
  FormationShape stringToFormationShape(const std::string & shape_str) const;
  std::string formationShapeToString(FormationShape shape) const;
  geometry_msgs::msg::PoseStamped makePoseFromOffset(double dx, double dy) const;
  void timerCallback();
  void publishState() const;
  void handleSetFormation(
    const my_custom_interfaces_pkg::srv::UpdateFormation::Request::SharedPtr request,
    my_custom_interfaces_pkg::srv::UpdateFormation::Response::SharedPtr response);

  std::vector<AgentPublisher> agent_publishers_;
  std::vector<std::string> agent_ids_;

  double publish_period_{0.1};
  std::string formation_shape_{"line"};
  double spacing_{0.0};
  double center_x_{5.0};
  double center_y_{5.0};
  double center_z_{0.5};
  double yaw_rad_{0.0};
  std::string frame_id_{"world"};

  // Runtime center (can be time-varying if motion is enabled)
  double center_x_runtime_{5.0};
  double center_y_runtime_{5.0};
  double center_z_runtime_{0.5};
  rclcpp::Time start_time_;
  bool started_{false};

  // Simple motion model for formation center (x-axis ramp) - LEGACY
  bool motion_enable_{false};
  double motion_vx_{0.0};
  double motion_start_x_{0.0};
  double motion_end_x_{0.0};

  // Waypoint-based trajectory system for zigzag patterns
  struct Waypoint {
    double t_start;  // Time to start moving to this waypoint (seconds)
    double t_end;    // Time to reach this waypoint (seconds)
    double x;
    double y;
    double z;
    FormationShape shape;  // Formation shape at this waypoint
  };

  bool waypoints_enable_{false};
  std::vector<Waypoint> waypoints_;
  size_t current_waypoint_idx_{0};

  // Shape transition management
  FormationShape current_shape_{FormationShape::TRIANGLE};
  FormationShape target_shape_{FormationShape::TRIANGLE};
  rclcpp::Time shape_transition_start_;
  bool in_shape_transition_{false};
  double shape_transition_duration_{3.0};  // 3 seconds for smooth transition
  std::vector<std::array<double, 2>> current_offsets_;
  std::vector<std::array<double, 2>> target_offsets_;

  void loadWaypoints();
  void updatePositionFromWaypoints(double elapsed_time);

  // Parametric trajectory support
  TrajectoryMode trajectory_mode_{TrajectoryMode::WAYPOINTS};
  double trajectory_radius_{5.0};       // Radius for circular/lemniscate
  double trajectory_period_{30.0};      // Period for one complete cycle (seconds)
  double trajectory_lemniscate_a_{8.0}; // Lemniscate X amplitude
  double trajectory_lemniscate_b_{4.0}; // Lemniscate Y amplitude
  double trajectory_spiral_rate_{0.5};  // Spiral expansion rate (m/cycle)

  TrajectoryMode stringToTrajectoryMode(const std::string& mode_str) const;
  void updatePositionFromTrajectory(double elapsed_time);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_custom_interfaces_pkg::msg::FormationState>::SharedPtr state_pub_;
  rclcpp::Service<my_custom_interfaces_pkg::srv::UpdateFormation>::SharedPtr set_formation_srv_;

  // =========================================================================
  // Event-Triggered Communication (ETC)
  // =========================================================================
  // When etc_enable_=false: time-triggered (original behavior, publish every cycle)
  // When etc_enable_=true:  event-triggered (publish only when conditions met)

  struct ETCState {
    geometry_msgs::msg::PoseStamped last_sent_pose;
    rclcpp::Time last_sent_time{0, 0, RCL_ROS_TIME};  // Explicit init to avoid undefined behavior
    bool initialized{false};
    uint64_t event_count{0};
  };

  struct ETCMetrics {
    uint64_t total_events{0};
    uint64_t total_cycles{0};
    double sum_inter_event_time{0.0};
    uint64_t inter_event_count{0};

    double eventRate() const {
      return total_cycles > 0 ? static_cast<double>(total_events) / static_cast<double>(total_cycles) : 0.0;
    }
    double avgInterEventTime() const {
      return inter_event_count > 0 ? sum_inter_event_time / static_cast<double>(inter_event_count) : 0.0;
    }
    double bandwidthReduction() const {
      return total_cycles > 0 ? 1.0 - eventRate() : 0.0;
    }
  };

  bool etc_enable_{false};          // DEFAULT false = time-triggered (original)
  double etc_epsilon_pos_{0.05};    // Position threshold [m]
  double etc_min_period_sec_{0.02}; // Anti-chattering min interval [s]
  double etc_max_period_sec_{0.5};  // Heartbeat max interval [s] (< stale threshold)

  std::unordered_map<std::string, ETCState> etc_agent_states_;
  ETCMetrics etc_metrics_;
  bool etc_force_next_publish_{false};  // Force publish on waypoint/shape change

  // ETC helper methods
  bool shouldTriggerEvent(const std::string& agent_id,
                          const geometry_msgs::msg::PoseStamped& current_pose);
  void publishETCMetrics();
  double poseDistance(const geometry_msgs::msg::PoseStamped& a,
                      const geometry_msgs::msg::PoseStamped& b) const;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr etc_metrics_pub_;
};

}  // namespace formation_coordinator_pkg

#endif  // FORMATION_COORDINATOR_PKG__FORMATION_COORDINATOR_NODE_HPP_
