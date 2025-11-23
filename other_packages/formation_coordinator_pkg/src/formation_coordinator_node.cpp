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

  // Legacy motion parameters
  declare_parameter<bool>("motion.enable", false);
  declare_parameter<double>("motion.vx", 0.0);
  declare_parameter<double>("motion.start_x", 0.0);
  declare_parameter<double>("motion.end_x", 0.0);

  // Waypoint-based trajectory
  declare_parameter<bool>("waypoints.enable", false);
  declare_parameter<std::vector<double>>("waypoints.times", std::vector<double>{});
  declare_parameter<std::vector<double>>("waypoints.x", std::vector<double>{});
  declare_parameter<std::vector<double>>("waypoints.y", std::vector<double>{});
  declare_parameter<std::vector<double>>("waypoints.z", std::vector<double>{});
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

  // Load legacy motion parameters
  motion_enable_ = get_parameter("motion.enable").as_bool();
  motion_vx_ = get_parameter("motion.vx").as_double();
  motion_start_x_ = get_parameter("motion.start_x").as_double();
  motion_end_x_ = get_parameter("motion.end_x").as_double();

  // Load waypoint parameters
  waypoints_enable_ = get_parameter("waypoints.enable").as_bool();
  if (waypoints_enable_) {
    loadWaypoints();
  }
}

void FormationCoordinatorNode::rebuildPublishers()
{
  agent_publishers_.clear();
  agent_publishers_.reserve(agent_ids_.size());

  for (const auto & agent_id : agent_ids_) {
    // Publish to absolute topics so agent namespaces receive them as intended
    // Use RELIABLE QoS to match controllers' subscribers
    auto pub = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/" + agent_id + "/target_pose", 10);
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

  pose.pose.position.x = center_x_runtime_ + rotated_x;
  pose.pose.position.y = center_y_runtime_ + rotated_y;
  pose.pose.position.z = center_z_runtime_;

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

  // Initialize on first call
  if (!started_) {
    started_ = true;
    start_time_ = now();
    center_x_runtime_ = motion_enable_ ? motion_start_x_ : center_x_;
    center_y_runtime_ = center_y_;
    center_z_runtime_ = center_z_;
  }

  const double elapsed = (now() - start_time_).seconds();

  // Priority: waypoints > legacy motion > static
  if (waypoints_enable_ && !waypoints_.empty()) {
    updatePositionFromWaypoints(elapsed);
  } else if (motion_enable_) {
    // Legacy linear motion along X
    center_x_runtime_ = motion_start_x_ + motion_vx_ * elapsed;
    // Clamp to end point based on direction
    if ((motion_vx_ >= 0.0 && center_x_runtime_ > motion_end_x_) ||
        (motion_vx_ < 0.0 && center_x_runtime_ < motion_end_x_)) {
      center_x_runtime_ = motion_end_x_;
    }
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
  msg.center_x = center_x_runtime_;
  msg.center_y = center_y_runtime_;
  msg.center_z = center_z_runtime_;
  msg.yaw_deg = yaw_rad_ * 180.0 / M_PI;
  msg.agent_ids = agent_ids_;
  state_pub_->publish(msg);
}

void FormationCoordinatorNode::loadWaypoints()
{
  auto times = get_parameter("waypoints.times").as_double_array();
  auto x_vals = get_parameter("waypoints.x").as_double_array();
  auto y_vals = get_parameter("waypoints.y").as_double_array();
  auto z_vals = get_parameter("waypoints.z").as_double_array();

  const size_t n = times.size();
  if (x_vals.size() != n || y_vals.size() != n || z_vals.size() != n) {
    RCLCPP_ERROR(get_logger(),
      "Waypoint array size mismatch: times=%zu, x=%zu, y=%zu, z=%zu",
      times.size(), x_vals.size(), y_vals.size(), z_vals.size());
    waypoints_enable_ = false;
    return;
  }

  if (n < 2) {
    RCLCPP_WARN(get_logger(), "Need at least 2 waypoints for trajectory");
    waypoints_enable_ = false;
    return;
  }

  waypoints_.clear();
  waypoints_.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    Waypoint wp;
    wp.t_start = (i == 0) ? 0.0 : times[i - 1];
    wp.t_end = times[i];
    wp.x = x_vals[i];
    wp.y = y_vals[i];
    wp.z = z_vals[i];
    waypoints_.push_back(wp);
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu waypoints for trajectory", waypoints_.size());
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    const auto & wp = waypoints_[i];
    RCLCPP_INFO(get_logger(), "  WP[%zu]: t=%.1f-%.1f s, pos=(%.2f, %.2f, %.2f)",
      i, wp.t_start, wp.t_end, wp.x, wp.y, wp.z);
  }
}

void FormationCoordinatorNode::updatePositionFromWaypoints(double elapsed_time)
{
  if (waypoints_.empty()) {
    return;
  }

  // Find current waypoint segment
  size_t idx = 0;
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    if (elapsed_time >= waypoints_[i].t_start && elapsed_time <= waypoints_[i].t_end) {
      idx = i;
      break;
    }
    if (i == waypoints_.size() - 1 && elapsed_time > waypoints_[i].t_end) {
      // Past all waypoints, hold final position
      idx = i;
      break;
    }
  }

  const auto & current_wp = waypoints_[idx];

  // Linear interpolation between waypoints
  double alpha = 0.0;
  if (idx == 0) {
    // First waypoint: interpolate from initial position
    const double duration = current_wp.t_end - current_wp.t_start;
    if (duration > 1e-6) {
      alpha = std::min(1.0, (elapsed_time - current_wp.t_start) / duration);
    } else {
      alpha = 1.0;
    }

    center_x_runtime_ = center_x_ + alpha * (current_wp.x - center_x_);
    center_y_runtime_ = center_y_ + alpha * (current_wp.y - center_y_);
    center_z_runtime_ = center_z_ + alpha * (current_wp.z - center_z_);

  } else {
    // Interpolate from previous waypoint
    const auto & prev_wp = waypoints_[idx - 1];
    const double duration = current_wp.t_end - current_wp.t_start;

    if (duration > 1e-6 && elapsed_time >= current_wp.t_start) {
      alpha = std::min(1.0, (elapsed_time - current_wp.t_start) / duration);
    } else if (elapsed_time < current_wp.t_start) {
      // Still at previous waypoint
      alpha = 0.0;
    } else {
      alpha = 1.0;
    }

    center_x_runtime_ = prev_wp.x + alpha * (current_wp.x - prev_wp.x);
    center_y_runtime_ = prev_wp.y + alpha * (current_wp.y - prev_wp.y);
    center_z_runtime_ = prev_wp.z + alpha * (current_wp.z - prev_wp.z);
  }
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

  // Reset runtime center and motion start
  started_ = false;
  center_x_runtime_ = center_x_;
  center_y_runtime_ = center_y_;

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
