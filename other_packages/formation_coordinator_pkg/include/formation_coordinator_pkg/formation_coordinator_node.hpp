#ifndef FORMATION_COORDINATOR_PKG__FORMATION_COORDINATOR_NODE_HPP_
#define FORMATION_COORDINATOR_PKG__FORMATION_COORDINATOR_NODE_HPP_

#include <array>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "my_custom_interfaces_pkg/msg/formation_state.hpp"
#include "my_custom_interfaces_pkg/srv/update_formation.hpp"
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

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_custom_interfaces_pkg::msg::FormationState>::SharedPtr state_pub_;
  rclcpp::Service<my_custom_interfaces_pkg::srv::UpdateFormation>::SharedPtr set_formation_srv_;
};

}  // namespace formation_coordinator_pkg

#endif  // FORMATION_COORDINATOR_PKG__FORMATION_COORDINATOR_NODE_HPP_
