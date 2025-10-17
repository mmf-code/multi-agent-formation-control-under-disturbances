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

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_custom_interfaces_pkg::msg::FormationState>::SharedPtr state_pub_;
  rclcpp::Service<my_custom_interfaces_pkg::srv::UpdateFormation>::SharedPtr set_formation_srv_;
};

}  // namespace formation_coordinator_pkg

#endif  // FORMATION_COORDINATOR_PKG__FORMATION_COORDINATOR_NODE_HPP_
