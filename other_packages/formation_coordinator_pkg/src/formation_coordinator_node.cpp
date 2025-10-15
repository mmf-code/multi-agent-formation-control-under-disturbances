#include "formation_coordinator_pkg/formation_coordinator_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace formation_coordinator_pkg
{
namespace
{
std::string to_lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}
}  // namespace

FormationCoordinatorNode::FormationCoordinatorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("formation_coordinator_node", options)
{
  declareParameters();
  loadParameters();
  rebuildPublishers();

  const double publish_rate_hz = publish_period_ > 0.0 ? 1.0 / publish_period_ : 10.0;
  RCLCPP_INFO(
    get_logger(), "Formation coordinator online: shape=%s spacing=%.2f m, rate=%.1f Hz",
    formation_shape_.c_str(), spacing_, publish_rate_hz);

  state_pub_ = create_publisher<my_custom_interfaces_pkg::msg::FormationState>("state", 10);

  set_formation_srv_ = create_service<my_custom_interfaces_pkg::srv::UpdateFormation>(
    "set_formation",
    std::bind(&FormationCoordinatorNode::handleSetFormation, this, std::placeholders::_1, std::placeholders::_2));

  if (publish_period_ <= 0.0) {
    publish_period_ = 0.1;
  }

  timer_ = create_wall_timer(
    std::chrono::duration<double>(publish_period_),
    std::bind(&FormationCoordinatorNode::timerCallback, this));
}

void FormationCoordinatorNode::declareParameters()
{
  declare_parameter<std::vector<std::string>>(
    "agent_ids", {"agent_0", "agent_1", "agent_2"});
  declare_parameter<std::string>("formation.shape", formation_shape_);
  declare_parameter<double>("formation.spacing", spacing_);
  declare_parameter<double>("formation.center.x", center_x_);
  declare_parameter<double>("formation.center.y", center_y_);
  declare_parameter<double>("formation.center.z", center_z_);
  declare_parameter<double>("formation.yaw_deg", 0.0);
  declare_parameter<std::string>("frame_id", frame_id_);
  declare_parameter<double>("publish_frequency_hz", 10.0);
}

void FormationCoordinatorNode::loadParameters()
{
  agent_ids_ = get_parameter("agent_ids").as_string_array();
  if (agent_ids_.empty()) {
    agent_ids_ = {"agent_0", "agent_1", "agent_2"};
    RCLCPP_WARN(get_logger(), "agent_ids parameter empty. Using default trio.");
  }

  formation_shape_ = to_lower(get_parameter("formation.shape").as_string());
  spacing_ = get_parameter("formation.spacing").as_double();
  center_x_ = get_parameter("formation.center.x").as_double();
  center_y_ = get_parameter("formation.center.y").as_double();
  center_z_ = get_parameter("formation.center.z").as_double();
  yaw_rad_ = get_parameter("formation.yaw_deg").as_double() * M_PI / 180.0;
  frame_id_ = get_parameter("frame_id").as_string();

  const double publish_frequency = get_parameter("publish_frequency_hz").as_double();
  publish_period_ = publish_frequency > 1e-6 ? 1.0 / publish_frequency : 0.1;
}

void FormationCoordinatorNode::rebuildPublishers()
{
  agent_publishers_.clear();
  agent_publishers_.reserve(agent_ids_.size());

  for (const auto & agent_id : agent_ids_) {
    auto pub = create_publisher<geometry_msgs::msg::PoseStamped>(agent_id + "/target_pose", 10);
    agent_publishers_.push_back(AgentPublisher{agent_id, pub});
  }
}

std::vector<std::array<double, 2>> FormationCoordinatorNode::computeOffsets(std::size_t count) const
{
  std::vector<std::array<double, 2>> offsets(count, {0.0, 0.0});
  if (count == 0) {
    return offsets;
  }

  const double s = std::max(spacing_, 0.1);

  if (formation_shape_ == "triangle") {
    if (count >= 1) {
      offsets[0] = {0.0, (std::sqrt(3.0) / 3.0) * s};
    }
    if (count >= 2) {
      offsets[1] = {-s / 2.0, -(std::sqrt(3.0) / 6.0) * s};
    }
    if (count >= 3) {
      offsets[2] = {s / 2.0, -(std::sqrt(3.0) / 6.0) * s};
    }
    for (std::size_t i = 3; i < count; ++i) {
      const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count);
      offsets[i] = {s * std::cos(angle), s * std::sin(angle)};
    }
    return offsets;
  }

  if (formation_shape_ == "line") {
    const double half_count = static_cast<double>(count - 1) / 2.0;
    for (std::size_t i = 0; i < count; ++i) {
      const double offset_index = static_cast<double>(i) - half_count;
      offsets[i] = {offset_index * s, 0.0};
    }
    return offsets;
  }

  if (formation_shape_ == "square") {
    const double half = s / 2.0;
    const std::vector<std::array<double, 2>> square_offsets = {
      {-half, half}, {half, half}, {-half, -half}, {half, -half}};
    for (std::size_t i = 0; i < count; ++i) {
      offsets[i] = square_offsets[i % square_offsets.size()];
    }
    return offsets;
  }

  RCLCPP_WARN_ONCE(
    get_logger(),
    "Unsupported formation shape '%s'. All offsets set to zero.", formation_shape_.c_str());
  return offsets;
}

geometry_msgs::msg::PoseStamped FormationCoordinatorNode::makePoseFromOffset(double dx, double dy) const
{
  const double cos_yaw = std::cos(yaw_rad_);
  const double sin_yaw = std::sin(yaw_rad_);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = now();
  pose.header.frame_id = frame_id_;

  const double rotated_x = cos_yaw * dx - sin_yaw * dy;
  const double rotated_y = sin_yaw * dx + cos_yaw * dy;

  pose.pose.position.x = center_x_ + rotated_x;
  pose.pose.position.y = center_y_ + rotated_y;
  pose.pose.position.z = center_z_;

  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(yaw_rad_ * 0.5);
  pose.pose.orientation.w = std::cos(yaw_rad_ * 0.5);
  return pose;
}

void FormationCoordinatorNode::timerCallback()
{
  if (agent_publishers_.empty()) {
    return;
  }

  const auto offsets = computeOffsets(agent_publishers_.size());
  for (std::size_t i = 0; i < agent_publishers_.size(); ++i) {
    const auto pose = makePoseFromOffset(offsets[i][0], offsets[i][1]);
    agent_publishers_[i].publisher->publish(pose);
  }

  publishState();
}

void FormationCoordinatorNode::publishState() const
{
  if (!state_pub_ || agent_publishers_.empty()) {
    return;
  }

  my_custom_interfaces_pkg::msg::FormationState msg;
  msg.shape = formation_shape_;
  msg.spacing = spacing_;
  msg.center_x = center_x_;
  msg.center_y = center_y_;
  msg.center_z = center_z_;
  msg.yaw_deg = yaw_rad_ * 180.0 / M_PI;
  msg.agent_ids = agent_ids_;
  state_pub_->publish(msg);
}

void FormationCoordinatorNode::handleSetFormation(
  const my_custom_interfaces_pkg::srv::UpdateFormation::Request::SharedPtr request,
  my_custom_interfaces_pkg::srv::UpdateFormation::Response::SharedPtr response)
{
  if (!request) {
    response->success = false;
    response->message = "Empty request received.";
    return;
  }

  const std::string requested_shape = to_lower(request->shape);
  if (!requested_shape.empty()) {
    formation_shape_ = requested_shape;
  }

  if (request->spacing > 1e-3) {
    spacing_ = request->spacing;
  }

  center_x_ = request->center_x;
  center_y_ = request->center_y;
  center_z_ = request->center_z;
  yaw_rad_ = request->yaw_deg * M_PI / 180.0;

  if (!request->agent_ids.empty()) {
    agent_ids_.assign(request->agent_ids.begin(), request->agent_ids.end());
  }

  // Reflect new settings in parameters for introspection tools.
  set_parameter(rclcpp::Parameter("formation.shape", formation_shape_));
  set_parameter(rclcpp::Parameter("formation.spacing", spacing_));
  set_parameter(rclcpp::Parameter("formation.center.x", center_x_));
  set_parameter(rclcpp::Parameter("formation.center.y", center_y_));
  set_parameter(rclcpp::Parameter("formation.center.z", center_z_));
  set_parameter(rclcpp::Parameter("formation.yaw_deg", request->yaw_deg));
  set_parameter(rclcpp::Parameter("agent_ids", agent_ids_));

  rebuildPublishers();

  response->success = true;
  response->message = "Formation updated.";

  RCLCPP_INFO(
    get_logger(),
    "Formation updated via service: shape=%s spacing=%.2f center=(%.2f, %.2f, %.2f) yaw=%.1f deg",
    formation_shape_.c_str(), spacing_, center_x_, center_y_, center_z_, request->yaw_deg);
}

}  // namespace formation_coordinator_pkg

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<formation_coordinator_pkg::FormationCoordinatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
