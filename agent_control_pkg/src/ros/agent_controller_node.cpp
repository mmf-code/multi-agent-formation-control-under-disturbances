#include "agent_control_pkg/agent_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace fs = std::filesystem;

namespace agent_control_pkg
{
namespace
{
std::string to_lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

double clamp_dt(double dt, double fallback)
{
  if (!std::isfinite(dt) || dt <= 0.0) {
    return fallback;
  }
  const double min_dt = 1e-4;
  const double max_dt = 0.25;
  return std::clamp(dt, min_dt, max_dt);
}
}  // namespace

AgentControllerNode::AgentControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("agent_controller_node", options)
{
  declareParameters();
  loadParameters();
  ensureFuzzyParamsLoaded();
  configureControllers();

  using std::placeholders::_1;
  target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "target_pose", rclcpp::SensorDataQoS(),
    std::bind(&AgentControllerNode::targetCallback, this, _1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&AgentControllerNode::odomCallback, this, _1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::Vector3>("cmd_accel", 10);

  if (publish_diagnostics_) {
    diag_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("diagnostics", 10);
  }

  if (loop_period_ <= 0.0) {
    loop_period_ = 0.005;
  }

  timer_ = create_wall_timer(
    std::chrono::duration<double>(loop_period_),
    std::bind(&AgentControllerNode::controlLoop, this));

  latest_target_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  latest_odom_ = std::make_shared<nav_msgs::msg::Odometry>();

  RCLCPP_INFO(
    get_logger(),
    "Agent controller node initialised: type=%s, dt=%.6f s, k_pid=%.2f, k_fuzzy=%.2f",
    controller_type_.c_str(), configured_dt_, mix_k_pid_, mix_k_fuzzy_);
}

void AgentControllerNode::declareParameters()
{
  declare_parameter<std::string>("controller_type", controller_type_);
  declare_parameter<double>("dt", configured_dt_);
  declare_parameter<double>("control_frequency_hz", 200.0);

  declare_parameter<double>("output_limits.x.min", axis_limits_x_.umin);
  declare_parameter<double>("output_limits.x.max", axis_limits_x_.umax);
  declare_parameter<double>("output_limits.y.min", axis_limits_y_.umin);
  declare_parameter<double>("output_limits.y.max", axis_limits_y_.umax);

  declare_parameter<double>("pid.kp", 0.6);
  declare_parameter<double>("pid.ki", 0.0);
  declare_parameter<double>("pid.kd", 0.0);
  declare_parameter<bool>("pid.enable_derivative_filter", true);
  declare_parameter<double>("pid.derivative_filter_alpha", 0.1);

  declare_parameter<bool>("fuzzy.enable", false);
  declare_parameter<bool>("fuzzy.include_wind", false);
  declare_parameter<double>("fuzzy.wind_scalar", 0.0);
  declare_parameter<std::string>("fuzzy.params_file", fuzzy_params_file_);

  declare_parameter<double>("mix.k_pid", 1.0);
  declare_parameter<double>("mix.k_fuzzy", 1.0);

  declare_parameter<bool>("diagnostics.enable", publish_diagnostics_);
}

void AgentControllerNode::loadParameters()
{
  controller_type_ = to_lower(get_parameter("controller_type").as_string());

  configured_dt_ = get_parameter("dt").as_double();
  const double control_frequency = get_parameter("control_frequency_hz").as_double();
  if (control_frequency > 0.0) {
    loop_period_ = 1.0 / control_frequency;
  } else {
    loop_period_ = configured_dt_;
  }
  if (configured_dt_ <= 0.0) {
    configured_dt_ = loop_period_;
  }
  if (configured_dt_ <= 0.0) {
    configured_dt_ = 0.005;
  }
  if (loop_period_ <= 0.0) {
    loop_period_ = configured_dt_;
  }

  axis_limits_x_.umin = get_parameter("output_limits.x.min").as_double();
  axis_limits_x_.umax = get_parameter("output_limits.x.max").as_double();
  axis_limits_y_.umin = get_parameter("output_limits.y.min").as_double();
  axis_limits_y_.umax = get_parameter("output_limits.y.max").as_double();

  pid_kp_ = get_parameter("pid.kp").as_double();
  pid_ki_ = get_parameter("pid.ki").as_double();
  pid_kd_ = get_parameter("pid.kd").as_double();
  pid_enable_derivative_filter_ = get_parameter("pid.enable_derivative_filter").as_bool();
  pid_derivative_filter_alpha_ = get_parameter("pid.derivative_filter_alpha").as_double();

  fuzzy_enable_ = get_parameter("fuzzy.enable").as_bool();
  fuzzy_include_wind_ = get_parameter("fuzzy.include_wind").as_bool();
  fuzzy_wind_scalar_ = get_parameter("fuzzy.wind_scalar").as_double();
  fuzzy_params_file_ = get_parameter("fuzzy.params_file").as_string();

  mix_k_pid_ = get_parameter("mix.k_pid").as_double();
  mix_k_fuzzy_ = get_parameter("mix.k_fuzzy").as_double();

  publish_diagnostics_ = get_parameter("diagnostics.enable").as_bool();
}

void AgentControllerNode::ensureFuzzyParamsLoaded()
{
  const bool needs_fuzzy = controller_type_ == "fuzzy" || controller_type_ == "pid_fuzzy";
  if (!(needs_fuzzy || fuzzy_enable_)) {
    return;
  }
  if (fuzzy_params_) {
    return;
  }

  std::string resolved_path = fuzzy_params_file_;
  try {
    if (!fs::path(resolved_path).is_absolute()) {
      const auto share_dir = ament_index_cpp::get_package_share_directory("agent_control_pkg");
      fs::path candidate = fs::path(share_dir) / "config" / resolved_path;
      if (fs::exists(candidate)) {
        resolved_path = candidate.string();
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_DEBUG(get_logger(), "Could not resolve package share directory: %s", ex.what());
  }

  if (!fs::exists(resolved_path)) {
    resolved_path = agent_control_pkg::findConfigFilePath(fuzzy_params_file_);
  }

  FuzzyParams params;
  if (!agent_control_pkg::loadFuzzyParamsYAML(resolved_path, params)) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to load fuzzy parameters from '%s'. Fuzzy controllers will be disabled.",
      resolved_path.c_str());
    fuzzy_enable_ = false;
    fuzzy_params_.reset();
    if (controller_type_ == "fuzzy" || controller_type_ == "pid_fuzzy") {
      controller_type_ = "pid";
    }
    return;
  }

  fuzzy_params_ = std::move(params);
  RCLCPP_INFO(get_logger(), "Loaded fuzzy parameters from '%s'", resolved_path.c_str());
}

void AgentControllerNode::configureControllers()
{
  axis_x_ = AxisController{};
  axis_y_ = AxisController{};

  axis_x_ = createAxisController("x", axis_limits_x_);
  axis_y_ = createAxisController("y", axis_limits_y_);

  if (!axis_x_.controller || !axis_y_.controller) {
    RCLCPP_ERROR(get_logger(), "Controller creation failed. Falling back to PID.");
    controller_type_ = "pid";
    axis_x_ = createAxisController("x", axis_limits_x_);
    axis_y_ = createAxisController("y", axis_limits_y_);
  }
}

AgentControllerNode::AxisController AgentControllerNode::createAxisController(
  const std::string & axis_name, const AxisLimits & limits)
{
  AxisController result;

  auto make_pid = [&](double kp, double ki, double kd) {
      auto pid = std::make_unique<controllers::PIDAdapter>(kp, ki, kd, limits.umin, limits.umax);
      pid->enableDerivativeFilter(pid_enable_derivative_filter_, pid_derivative_filter_alpha_);
      result.pid = pid.get();
      result.controller = std::move(pid);
    };

  auto make_fuzzy = [&]() -> std::unique_ptr<controllers::FuzzyGT2Adapter> {
      controllers::FuzzyGT2Adapter::Options opt;
      opt.umin = limits.umin;
      opt.umax = limits.umax;
      opt.wind_scalar = fuzzy_wind_scalar_;
      auto fuzzy = std::make_unique<controllers::FuzzyGT2Adapter>(opt);
      if (fuzzy_params_) {
        fuzzy->configureFromFuzzyParams(*fuzzy_params_, fuzzy_include_wind_);
      }
      result.fuzzy = fuzzy.get();
      return fuzzy;
    };

  const std::string type = controller_type_;
  if (type == "pid" || type == "p" || type == "pi" || type == "pd") {
    double kp = pid_kp_;
    double ki = pid_ki_;
    double kd = pid_kd_;

    if (type == "p") {
      ki = 0.0;
      kd = 0.0;
    } else if (type == "pi") {
      kd = 0.0;
    } else if (type == "pd") {
      ki = 0.0;
    }

    make_pid(kp, ki, kd);
    return result;
  }

  if (type == "fuzzy") {
    if (!fuzzy_params_) {
      RCLCPP_WARN(
        get_logger(),
        "Axis '%s' requested fuzzy controller but parameters are unavailable. Using PID fallback.",
        axis_name.c_str());
      make_pid(pid_kp_, pid_ki_, pid_kd_);
      return result;
    }
    auto fuzzy = make_fuzzy();
    result.controller = std::move(fuzzy);
    return result;
  }

  if (type == "pid_fuzzy") {
    if (!fuzzy_params_) {
      RCLCPP_WARN(
        get_logger(),
        "Axis '%s' requested hybrid controller but fuzzy parameters are unavailable. Using PID fallback.",
        axis_name.c_str());
      make_pid(pid_kp_, pid_ki_, pid_kd_);
      return result;
    }

    auto pid = std::make_unique<controllers::PIDAdapter>(pid_kp_, pid_ki_, pid_kd_, limits.umin, limits.umax);
    pid->enableDerivativeFilter(pid_enable_derivative_filter_, pid_derivative_filter_alpha_);
    auto raw_pid = pid.get();

    auto fuzzy = make_fuzzy();
    auto raw_fuzzy = fuzzy.get();

    auto combined = std::make_unique<controllers::CombinedPidFuzzyAdapter>(
      std::move(pid), std::move(fuzzy), mix_k_pid_, mix_k_fuzzy_, limits.umin, limits.umax);

    result.pid = raw_pid;
    result.fuzzy = raw_fuzzy;
    result.combined = combined.get();
    result.controller = std::move(combined);
    return result;
  }

  RCLCPP_WARN(
    get_logger(), "Unknown controller_type '%s'. Falling back to PID.", controller_type_.c_str());
  make_pid(pid_kp_, pid_ki_, pid_kd_);
  return result;
}

void AgentControllerNode::targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  *latest_target_ = *msg;
  has_target_.store(true, std::memory_order_release);
}

void AgentControllerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  *latest_odom_ = *msg;
  has_odom_.store(true, std::memory_order_release);
}

void AgentControllerNode::controlLoop()
{
  if (!axis_x_.controller || !axis_y_.controller) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Controllers not configured yet. Skipping control loop.");
    return;
  }

  if (!has_target_.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No target received yet. Awaiting target_pose.");
    return;
  }
  if (!has_odom_.load(std::memory_order_acquire)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No odometry received yet. Awaiting odom.");
    return;
  }

  geometry_msgs::msg::PoseStamped target;
  nav_msgs::msg::Odometry odom;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target = *latest_target_;
    odom = *latest_odom_;
  }

  rclcpp::Time now = this->now();
  double dt = configured_dt_;

  if (!first_control_cycle_) {
    const double actual_dt = (now - last_control_time_).seconds();
    dt = configured_dt_ > 0.0 ? configured_dt_ : actual_dt;
    dt = clamp_dt(dt, configured_dt_);
  } else {
    dt = configured_dt_;
    first_control_cycle_ = false;
  }
  last_control_time_ = now;

  const double current_x = odom.pose.pose.position.x;
  const double current_y = odom.pose.pose.position.y;
  const double target_x = target.pose.position.x;
  const double target_y = target.pose.position.y;

  double ax = axis_x_.controller->compute(current_x, target_x, dt);
  double ay = axis_y_.controller->compute(current_y, target_y, dt);

  axis_x_.last_total_output = ax;
  axis_y_.last_total_output = ay;

  if (axis_x_.combined) {
    axis_x_.last_pid_contribution = axis_x_.combined->lastPidContribution();
    axis_x_.last_fuzzy_contribution = axis_x_.combined->lastFuzzyContribution();
  } else if (axis_x_.pid && !axis_x_.fuzzy) {
    axis_x_.last_pid_contribution = axis_x_.pid->pid().getLastTerms().total_output;
    axis_x_.last_fuzzy_contribution = 0.0;
  } else if (axis_x_.fuzzy && !axis_x_.pid) {
    axis_x_.last_pid_contribution = 0.0;
    axis_x_.last_fuzzy_contribution = ax;
  } else if (axis_x_.pid && axis_x_.fuzzy) {
    axis_x_.last_pid_contribution = axis_x_.pid->pid().getLastTerms().total_output;
    axis_x_.last_fuzzy_contribution = ax - axis_x_.last_pid_contribution;
  }

  if (axis_y_.combined) {
    axis_y_.last_pid_contribution = axis_y_.combined->lastPidContribution();
    axis_y_.last_fuzzy_contribution = axis_y_.combined->lastFuzzyContribution();
  } else if (axis_y_.pid && !axis_y_.fuzzy) {
    axis_y_.last_pid_contribution = axis_y_.pid->pid().getLastTerms().total_output;
    axis_y_.last_fuzzy_contribution = 0.0;
  } else if (axis_y_.fuzzy && !axis_y_.pid) {
    axis_y_.last_pid_contribution = 0.0;
    axis_y_.last_fuzzy_contribution = ay;
  } else if (axis_y_.pid && axis_y_.fuzzy) {
    axis_y_.last_pid_contribution = axis_y_.pid->pid().getLastTerms().total_output;
    axis_y_.last_fuzzy_contribution = ay - axis_y_.last_pid_contribution;
  }

  geometry_msgs::msg::Vector3 cmd_msg;
  cmd_msg.x = ax;
  cmd_msg.y = ay;
  cmd_msg.z = 0.0;
  cmd_pub_->publish(cmd_msg);

  publishDiagnostics();
}

void AgentControllerNode::publishDiagnostics()
{
  if (!publish_diagnostics_ || !diag_pub_) {
    return;
  }
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    axis_x_.last_total_output,
    axis_x_.last_pid_contribution,
    axis_x_.last_fuzzy_contribution,
    axis_y_.last_total_output,
    axis_y_.last_pid_contribution,
    axis_y_.last_fuzzy_contribution
  };
  diag_pub_->publish(msg);
}

}  // namespace agent_control_pkg

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agent_control_pkg::AgentControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
