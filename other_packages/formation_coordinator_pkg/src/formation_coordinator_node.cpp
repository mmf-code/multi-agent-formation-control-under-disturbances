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

FormationShape FormationCoordinatorNode::stringToFormationShape(const std::string & shape_str) const
{
  const std::string lower_shape = to_lower(shape_str);

  if (lower_shape == "triangle") {
    return FormationShape::TRIANGLE;
  } else if (lower_shape == "line") {
    return FormationShape::LINE;
  } else if (lower_shape == "square") {
    return FormationShape::SQUARE;
  } else if (lower_shape == "v_shape" || lower_shape == "v-shape" || lower_shape == "vshape") {
    return FormationShape::V_SHAPE;
  }

  // Default to triangle if unknown
  RCLCPP_WARN_ONCE(get_logger(), "Unknown formation shape '%s', defaulting to triangle", shape_str.c_str());
  return FormationShape::TRIANGLE;
}

std::string FormationCoordinatorNode::formationShapeToString(FormationShape shape) const
{
  switch (shape) {
    case FormationShape::TRIANGLE:
      return "triangle";
    case FormationShape::LINE:
      return "line";
    case FormationShape::SQUARE:
      return "square";
    case FormationShape::V_SHAPE:
      return "v_shape";
    default:
      return "triangle";
  }
}

std::vector<std::array<double, 2>> FormationCoordinatorNode::generateFormationPositions(
  FormationShape shape, std::size_t count) const
{
  std::vector<std::array<double, 2>> positions(count, {0.0, 0.0});
  if (count == 0) {
    return positions;
  }

  const double s = std::max(spacing_, 0.1);

  switch (shape) {
    case FormationShape::TRIANGLE:
      // Equilateral triangle with centroid at origin
      if (count >= 1) {
        positions[0] = {0.0, (std::sqrt(3.0) / 3.0) * s};
      }
      if (count >= 2) {
        positions[1] = {-s / 2.0, -(std::sqrt(3.0) / 6.0) * s};
      }
      if (count >= 3) {
        positions[2] = {s / 2.0, -(std::sqrt(3.0) / 6.0) * s};
      }
      // Extra agents in circular pattern
      for (std::size_t i = 3; i < count; ++i) {
        const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count);
        positions[i] = {s * std::cos(angle), s * std::sin(angle)};
      }
      break;

    case FormationShape::LINE:
      // Horizontal line centered at origin
      {
        const double half_count = static_cast<double>(count - 1) / 2.0;
        for (std::size_t i = 0; i < count; ++i) {
          const double offset_index = static_cast<double>(i) - half_count;
          positions[i] = {offset_index * s, 0.0};
        }
      }
      break;

    case FormationShape::SQUARE:
      // L-shape for 3 drones (incomplete square)
      if (count >= 1) {
        positions[0] = {-s / 2.0, s / 2.0};   // Top-left
      }
      if (count >= 2) {
        positions[1] = {s / 2.0, s / 2.0};    // Top-right
      }
      if (count >= 3) {
        positions[2] = {-s / 2.0, -s / 2.0};  // Bottom-left
      }
      // Extra agents complete the square pattern
      if (count >= 4) {
        positions[3] = {s / 2.0, -s / 2.0};   // Bottom-right
      }
      for (std::size_t i = 4; i < count; ++i) {
        // Additional agents in circular pattern around square
        const double angle = 2.0 * M_PI * static_cast<double>(i - 4) / static_cast<double>(count - 4);
        positions[i] = {s * 1.5 * std::cos(angle), s * 1.5 * std::sin(angle)};
      }
      break;

    case FormationShape::V_SHAPE:
      // V formation: leader at front, wings behind
      if (count >= 1) {
        positions[0] = {0.0, s * 0.5};           // Leader (front center)
      }
      if (count >= 2) {
        positions[1] = {-s * 0.7, -s * 0.5};     // Left wing
      }
      if (count >= 3) {
        positions[2] = {s * 0.7, -s * 0.5};      // Right wing
      }
      // Extra agents extend the V pattern
      for (std::size_t i = 3; i < count; ++i) {
        const bool is_left_wing = (i % 2 == 1);
        const double wing_index = static_cast<double>((i - 3) / 2 + 2);
        const double x_offset = is_left_wing ? -s * 0.7 * wing_index : s * 0.7 * wing_index;
        const double y_offset = -s * 0.5 * wing_index;
        positions[i] = {x_offset, y_offset};
      }
      break;

    default:
      RCLCPP_WARN_ONCE(get_logger(), "Unsupported formation shape in generateFormationPositions");
      break;
  }

  return positions;
}

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

  // Log ETC configuration
  if (etc_enable_) {
    RCLCPP_INFO(get_logger(),
      "ETC enabled: epsilon_pos=%.3f m, min_period=%.3f s, max_period=%.3f s",
      etc_epsilon_pos_, etc_min_period_sec_, etc_max_period_sec_);
  } else {
    RCLCPP_INFO(get_logger(), "ETC disabled: using time-triggered communication");
  }

  state_pub_ = create_publisher<my_custom_interfaces_pkg::msg::FormationState>("state", 10);

  // ETC metrics publisher (publishes even when ETC disabled for comparison)
  etc_metrics_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("etc_metrics", 10);

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
  declare_parameter<std::vector<std::string>>("waypoints.shapes", std::vector<std::string>{});

  // Shape transition parameters
  declare_parameter<double>("shape_transition_duration", shape_transition_duration_);

  // Event-Triggered Communication (ETC) parameters
  // When etc.enable=false: time-triggered (original behavior)
  // When etc.enable=true:  event-triggered (publish only when needed)
  declare_parameter<bool>("etc.enable", etc_enable_);
  declare_parameter<double>("etc.epsilon_pos", etc_epsilon_pos_);
  declare_parameter<double>("etc.min_period_sec", etc_min_period_sec_);
  declare_parameter<double>("etc.max_period_sec", etc_max_period_sec_);
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

  // Load shape transition duration
  shape_transition_duration_ = get_parameter("shape_transition_duration").as_double();
  if (shape_transition_duration_ < 0.5) {
    shape_transition_duration_ = 3.0;  // Minimum 0.5s, default 3.0s
    RCLCPP_WARN(get_logger(), "Shape transition duration too short, using default 3.0s");
  }

  // Initialize current shape from default formation shape
  current_shape_ = stringToFormationShape(formation_shape_);
  target_shape_ = current_shape_;

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

  // Load ETC parameters
  etc_enable_ = get_parameter("etc.enable").as_bool();
  etc_epsilon_pos_ = get_parameter("etc.epsilon_pos").as_double();
  etc_min_period_sec_ = get_parameter("etc.min_period_sec").as_double();
  etc_max_period_sec_ = get_parameter("etc.max_period_sec").as_double();

  // Validate: max_period must be less than agent controller's stale threshold (2.0s)
  if (etc_max_period_sec_ >= 2.0) {
    RCLCPP_WARN(get_logger(),
      "ETC max_period_sec (%.2f) >= stale threshold (2.0s). Clamping to 1.5s.",
      etc_max_period_sec_);
    etc_max_period_sec_ = 1.5;
  }
  if (etc_min_period_sec_ >= etc_max_period_sec_) {
    RCLCPP_WARN(get_logger(),
      "ETC min_period_sec (%.3f) >= max_period_sec (%.3f). Setting min to max/10.",
      etc_min_period_sec_, etc_max_period_sec_);
    etc_min_period_sec_ = etc_max_period_sec_ / 10.0;
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

  // If in transition, interpolate between current and target shapes
  if (in_shape_transition_) {
    const double elapsed = (now() - shape_transition_start_).seconds();
    const double progress = std::min(1.0, elapsed / shape_transition_duration_);

    // Linear interpolation (LERP) between current and target offsets
    for (std::size_t i = 0; i < count; ++i) {
      if (i < current_offsets_.size() && i < target_offsets_.size()) {
        offsets[i][0] = current_offsets_[i][0] + progress * (target_offsets_[i][0] - current_offsets_[i][0]);
        offsets[i][1] = current_offsets_[i][1] + progress * (target_offsets_[i][1] - current_offsets_[i][1]);
      }
    }

    // Transition complete
    if (progress >= 1.0) {
      const_cast<FormationCoordinatorNode*>(this)->in_shape_transition_ = false;
      const_cast<FormationCoordinatorNode*>(this)->current_shape_ = target_shape_;
      const_cast<FormationCoordinatorNode*>(this)->current_offsets_ = target_offsets_;
      RCLCPP_INFO(get_logger(), "Shape transition complete: %s",
        formationShapeToString(current_shape_).c_str());
    }

    return offsets;
  }

  // No transition, return current shape positions
  return generateFormationPositions(current_shape_, count);
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

  // Track cycle for ETC metrics
  etc_metrics_.total_cycles++;
  bool any_event_this_cycle = false;

  for (std::size_t i = 0; i < agent_publishers_.size(); ++i) {
    const auto pose = makePoseFromOffset(offsets[i][0], offsets[i][1]);
    const std::string& agent_id = agent_publishers_[i].id;

    if (etc_enable_) {
      // Event-Triggered Communication
      if (shouldTriggerEvent(agent_id, pose)) {
        agent_publishers_[i].publisher->publish(pose);
        any_event_this_cycle = true;
      }
    } else {
      // Time-Triggered Communication (original behavior)
      agent_publishers_[i].publisher->publish(pose);
      any_event_this_cycle = true;

      // Still track for metrics comparison
      auto& state = etc_agent_states_[agent_id];
      if (!state.initialized) {
        state.last_sent_time = now();
        state.initialized = true;
      }
      state.event_count++;
    }
  }

  // Reset force flag after use
  if (etc_force_next_publish_) {
    etc_force_next_publish_ = false;
  }

  if (any_event_this_cycle) {
    etc_metrics_.total_events++;
  }

  publishState();
  publishETCMetrics();
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
  auto shape_strs = get_parameter("waypoints.shapes").as_string_array();

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

  // If shapes array is empty or size mismatch, use default shape for all waypoints
  bool use_default_shapes = shape_strs.empty() || shape_strs.size() != n;
  if (use_default_shapes && !shape_strs.empty()) {
    RCLCPP_WARN(get_logger(),
      "Waypoint shapes array size mismatch (%zu vs %zu), using default shape for all waypoints",
      shape_strs.size(), n);
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

    // Set shape for this waypoint
    if (use_default_shapes) {
      wp.shape = current_shape_;  // Use default formation shape
    } else {
      wp.shape = stringToFormationShape(shape_strs[i]);
    }

    waypoints_.push_back(wp);
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu waypoints for trajectory", waypoints_.size());
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    const auto & wp = waypoints_[i];
    RCLCPP_INFO(get_logger(), "  WP[%zu]: t=%.1f-%.1f s, pos=(%.2f, %.2f, %.2f), shape=%s",
      i, wp.t_start, wp.t_end, wp.x, wp.y, wp.z, formationShapeToString(wp.shape).c_str());
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

  // Check if waypoint changed and trigger shape transition if needed
  if (idx != current_waypoint_idx_) {
    current_waypoint_idx_ = idx;

    // Force ETC publish on waypoint change
    etc_force_next_publish_ = true;

    // Check if shape needs to change
    if (current_wp.shape != target_shape_) {
      // Start shape transition
      target_shape_ = current_wp.shape;
      current_offsets_ = generateFormationPositions(current_shape_, agent_publishers_.size());
      target_offsets_ = generateFormationPositions(target_shape_, agent_publishers_.size());
      in_shape_transition_ = true;
      shape_transition_start_ = now();

      RCLCPP_INFO(get_logger(),
        "Starting shape transition: %s -> %s (waypoint %zu)",
        formationShapeToString(current_shape_).c_str(),
        formationShapeToString(target_shape_).c_str(),
        idx);
    }
  }

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

    // Trigger shape transition if shape changed
    FormationShape new_shape = stringToFormationShape(requested_shape);
    if (new_shape != current_shape_ && !in_shape_transition_) {
      target_shape_ = new_shape;
      current_offsets_ = generateFormationPositions(current_shape_, agent_publishers_.size());
      target_offsets_ = generateFormationPositions(target_shape_, agent_publishers_.size());
      in_shape_transition_ = true;
      shape_transition_start_ = now();

      // Force ETC publish on shape change
      etc_force_next_publish_ = true;

      RCLCPP_INFO(get_logger(),
        "Service triggered shape transition: %s -> %s",
        formationShapeToString(current_shape_).c_str(),
        formationShapeToString(target_shape_).c_str());
    }
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

// =========================================================================
// Event-Triggered Communication (ETC) Implementation
// =========================================================================

double FormationCoordinatorNode::poseDistance(
  const geometry_msgs::msg::PoseStamped& a,
  const geometry_msgs::msg::PoseStamped& b) const
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  const double dz = a.pose.position.z - b.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool FormationCoordinatorNode::shouldTriggerEvent(
  const std::string& agent_id,
  const geometry_msgs::msg::PoseStamped& current_pose)
{
  auto& state = etc_agent_states_[agent_id];
  const rclcpp::Time current_time = now();

  // First message for this agent: always trigger
  if (!state.initialized) {
    state.last_sent_pose = current_pose;
    state.last_sent_time = current_time;
    state.initialized = true;
    state.event_count = 1;
    return true;
  }

  const double time_since_last = (current_time - state.last_sent_time).seconds();

  // Condition 1: Anti-chattering (min_period not elapsed)
  if (time_since_last < etc_min_period_sec_) {
    return false;
  }

  bool should_trigger = false;
  std::string trigger_reason;

  // Condition 2: Forced publish (waypoint/shape change)
  if (etc_force_next_publish_) {
    should_trigger = true;
    trigger_reason = "forced";
  }

  // Condition 3: Position change exceeds threshold
  if (!should_trigger) {
    const double distance = poseDistance(current_pose, state.last_sent_pose);
    if (distance > etc_epsilon_pos_) {
      should_trigger = true;
      trigger_reason = "position";
    }
  }

  // Condition 4: Heartbeat (max_period exceeded)
  if (!should_trigger && time_since_last >= etc_max_period_sec_) {
    should_trigger = true;
    trigger_reason = "heartbeat";
  }

  if (should_trigger) {
    // Update inter-event time metrics
    etc_metrics_.sum_inter_event_time += time_since_last;
    etc_metrics_.inter_event_count++;

    // Update state
    state.last_sent_pose = current_pose;
    state.last_sent_time = current_time;
    state.event_count++;

    RCLCPP_DEBUG(get_logger(), "ETC event [%s]: %s (dt=%.3fs, total=%lu)",
      agent_id.c_str(), trigger_reason.c_str(), time_since_last, state.event_count);
  }

  return should_trigger;
}

void FormationCoordinatorNode::publishETCMetrics()
{
  if (!etc_metrics_pub_) {
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  // Data format: [etc_enable, total_events, total_cycles, event_rate, avg_inter_event_time, bandwidth_reduction]
  msg.data = {
    etc_enable_ ? 1.0 : 0.0,
    static_cast<double>(etc_metrics_.total_events),
    static_cast<double>(etc_metrics_.total_cycles),
    etc_metrics_.eventRate(),
    etc_metrics_.avgInterEventTime(),
    etc_metrics_.bandwidthReduction()
  };
  etc_metrics_pub_->publish(msg);
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
