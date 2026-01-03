#include "agent_control_pkg/agent_controller_node.hpp"
#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"

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
    "target_pose", 10,
    std::bind(&AgentControllerNode::targetCallback, this, _1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&AgentControllerNode::odomCallback, this, _1));

  wind_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    wind_source_topic_, 10,
    std::bind(&AgentControllerNode::windCallback, this, _1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::Vector3>("cmd_accel", 10);

  if (publish_diagnostics_) {
    diag_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("diagnostics", 10);
  }

  // Controller parameters publisher (for dashboard monitoring)
  params_pub_ = create_publisher<my_custom_interfaces_pkg::msg::ControllerParams>("controller_params", 10);

#ifdef CRAZYFLIE_SUPPORT
  // Crazyflie dual-output publisher (when enabled)
  if (crazyflie_enable_) {
    cf_cmd_pub_ = create_publisher<crazyflie_interfaces::msg::FullState>(crazyflie_cmd_topic_, 10);
    RCLCPP_INFO(get_logger(), "Crazyflie dual-output enabled: topic='%s'", crazyflie_cmd_topic_.c_str());
  }
#endif

  if (loop_period_ <= 0.0) {
    loop_period_ = 0.005;
  }

  timer_ = create_wall_timer(
    std::chrono::duration<double>(loop_period_),
    std::bind(&AgentControllerNode::controlLoop, this));

  // Publish controller parameters every 2 seconds
  params_timer_ = create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&AgentControllerNode::publishControllerParams, this));

  // Optional debug logging of fuzzy wind scalar for this agent
  if (debug_wind_input_) {
    wind_debug_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AgentControllerNode::windDebugTimerCallback, this));
    RCLCPP_INFO(
      get_logger(),
      "Wind debug input enabled: topic='%s', type='%s'",
      wind_source_topic_.c_str(), wind_source_type_.c_str());
  }

  latest_target_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
  latest_odom_ = std::make_shared<nav_msgs::msg::Odometry>();

  RCLCPP_INFO(
    get_logger(),
    "Agent controller node initialised: type=%s, dt=%.6f s, PID[Kp=%.3f, Ki=%.3f, Kd=%.3f], mix[k_pid=%.2f, k_fuzzy=%.2f]",
    controller_type_.c_str(), configured_dt_, pid_kp_, pid_ki_, pid_kd_, mix_k_pid_, mix_k_fuzzy_);
}

void AgentControllerNode::declareParameters()
{
  declare_parameter<std::string>("controller_type", controller_type_);
  declare_parameter<double>("dt", configured_dt_);
  declare_parameter<double>("control_frequency_hz", 200.0);

  // Wind source configuration for fuzzy and feed-forward handling
  declare_parameter<std::string>("wind_source_topic", wind_source_topic_);
  declare_parameter<std::string>("wind_source_type", wind_source_type_);
  declare_parameter<bool>("debug_wind_input", debug_wind_input_);

  declare_parameter<double>("output_limits.x.min", axis_limits_x_.umin);
  declare_parameter<double>("output_limits.x.max", axis_limits_x_.umax);
  declare_parameter<double>("output_limits.y.min", axis_limits_y_.umin);
  declare_parameter<double>("output_limits.y.max", axis_limits_y_.umax);

  declare_parameter<double>("pid.kp", 0.6);
  declare_parameter<double>("pid.ki", 0.0);
  declare_parameter<double>("pid.kd", 0.0);
  declare_parameter<bool>("pid.enable_derivative_filter", true);
  declare_parameter<double>("pid.derivative_filter_alpha", 0.1);
  declare_parameter<std::string>("pid.anti_windup_mode", "combined");
  declare_parameter<double>("pid.tracking_time_constant", 0.0);

  declare_parameter<bool>("fuzzy.enable", false);
  declare_parameter<bool>("fuzzy.include_wind", false);
  declare_parameter<double>("fuzzy.wind_scalar", 0.0);
  declare_parameter<std::string>("fuzzy.params_file", fuzzy_params_file_);

  declare_parameter<double>("mix.k_pid", 1.0);
  declare_parameter<double>("mix.k_fuzzy", 1.0);

  // GT2 Fuzzy specific parameters
  declare_parameter<int>("gt2.num_alpha_levels", 5);
  declare_parameter<std::string>("gt2.secondary_shape", "triangular");
  declare_parameter<double>("gt2.secondary_spread", 0.3);
  declare_parameter<std::string>("gt2.params_file", "gt2_fuzzy_params_crazyflie.yaml");

  declare_parameter<bool>("diagnostics.enable", publish_diagnostics_);

  // Data freshness (stale data protection) parameters
  declare_parameter<bool>("stale_data.enable", true);
  declare_parameter<double>("stale_data.odom_threshold_sec", 0.5);
  declare_parameter<double>("stale_data.target_threshold_sec", 2.0);

  // Feed-forward parameters
  declare_parameter<bool>("feedforward.enable_drag", false);
  declare_parameter<bool>("feedforward.enable_wind", false);
  declare_parameter<double>("feedforward.k_drag", 0.8);
  declare_parameter<double>("feedforward.k_wind", 1.0);
  declare_parameter<double>("feedforward.mass_estimate", 1.5);
  declare_parameter<double>("feedforward.drag_coeff_lin_estimate", 0.12);
  declare_parameter<double>("feedforward.drag_coeff_quad_estimate", 0.05);
  declare_parameter<double>("feedforward.drag_speed_threshold_estimate", 1.0);

#ifdef CRAZYFLIE_SUPPORT
  // Crazyflie dual-output parameters
  declare_parameter<bool>("crazyflie.enable", false);
  declare_parameter<std::string>("crazyflie.cmd_topic", "cmd_full_state");

  // Crazyflie attitude converter parameters
  declare_parameter<double>("crazyflie.mass", 0.027);
  declare_parameter<double>("crazyflie.max_thrust", 0.6);
  declare_parameter<double>("crazyflie.max_roll_rad", 0.5236);
  declare_parameter<double>("crazyflie.max_pitch_rad", 0.5236);
#endif
}

void AgentControllerNode::loadParameters()
{
  controller_type_ = to_lower(get_parameter("controller_type").as_string());

  // Wind source configuration
  wind_source_topic_ = get_parameter("wind_source_topic").as_string();
  wind_source_type_ = to_lower(get_parameter("wind_source_type").as_string());
  debug_wind_input_ = get_parameter("debug_wind_input").as_bool();

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
  pid_anti_windup_mode_ = get_parameter("pid.anti_windup_mode").as_string();
  pid_tracking_time_constant_ = get_parameter("pid.tracking_time_constant").as_double();

  fuzzy_enable_ = get_parameter("fuzzy.enable").as_bool();
  fuzzy_include_wind_ = get_parameter("fuzzy.include_wind").as_bool();
  fuzzy_wind_scalar_ = get_parameter("fuzzy.wind_scalar").as_double();
  fuzzy_params_file_ = get_parameter("fuzzy.params_file").as_string();

  mix_k_pid_ = get_parameter("mix.k_pid").as_double();
  mix_k_fuzzy_ = get_parameter("mix.k_fuzzy").as_double();

  // Load GT2 parameters
  gt2_num_alpha_levels_ = get_parameter("gt2.num_alpha_levels").as_int();
  gt2_secondary_shape_ = to_lower(get_parameter("gt2.secondary_shape").as_string());
  gt2_secondary_spread_ = get_parameter("gt2.secondary_spread").as_double();
  gt2_params_file_ = get_parameter("gt2.params_file").as_string();

  publish_diagnostics_ = get_parameter("diagnostics.enable").as_bool();

  // Load stale data protection parameters
  enable_stale_data_check_ = get_parameter("stale_data.enable").as_bool();
  odom_stale_threshold_sec_ = get_parameter("stale_data.odom_threshold_sec").as_double();
  target_stale_threshold_sec_ = get_parameter("stale_data.target_threshold_sec").as_double();

  // Load feed-forward parameters
  ff_params_.enable_drag_ff = get_parameter("feedforward.enable_drag").as_bool();
  ff_params_.enable_wind_ff = get_parameter("feedforward.enable_wind").as_bool();
  ff_params_.k_drag = get_parameter("feedforward.k_drag").as_double();
  ff_params_.k_wind = get_parameter("feedforward.k_wind").as_double();
  ff_params_.mass_estimate = get_parameter("feedforward.mass_estimate").as_double();
  ff_params_.drag_coeff_lin_estimate = get_parameter("feedforward.drag_coeff_lin_estimate").as_double();
  ff_params_.drag_coeff_quad_estimate = get_parameter("feedforward.drag_coeff_quad_estimate").as_double();
  ff_params_.drag_speed_threshold_estimate = get_parameter("feedforward.drag_speed_threshold_estimate").as_double();

#ifdef CRAZYFLIE_SUPPORT
  // Load Crazyflie dual-output parameters
  crazyflie_enable_ = get_parameter("crazyflie.enable").as_bool();
  crazyflie_cmd_topic_ = get_parameter("crazyflie.cmd_topic").as_string();

  // Load attitude converter parameters
  cf_attitude_config_.mass = get_parameter("crazyflie.mass").as_double();
  cf_attitude_config_.max_thrust = get_parameter("crazyflie.max_thrust").as_double();
  cf_attitude_config_.max_roll_rad = get_parameter("crazyflie.max_roll_rad").as_double();
  cf_attitude_config_.max_pitch_rad = get_parameter("crazyflie.max_pitch_rad").as_double();
  cf_attitude_config_.hover_thrust = cf_attitude_config_.mass * cf_attitude_config_.gravity;

  // Initialize the converter
  accel_to_attitude_ = std::make_unique<core::AccelToAttitude>(cf_attitude_config_);
#endif
}

void AgentControllerNode::ensureFuzzyParamsLoaded()
{
  const bool needs_it2_fuzzy = controller_type_ == "fuzzy" || controller_type_ == "pid_fuzzy";
  const bool needs_gt2_fuzzy = controller_type_ == "gt2_fuzzy" || controller_type_ == "pid_gt2_fuzzy";

  // Load IT2 fuzzy params if needed
  if ((needs_it2_fuzzy || fuzzy_enable_) && !fuzzy_params_) {
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
        "Failed to load IT2 fuzzy parameters from '%s'. Fuzzy controllers will be disabled.",
        resolved_path.c_str());
      fuzzy_enable_ = false;
      fuzzy_params_.reset();
      if (controller_type_ == "fuzzy" || controller_type_ == "pid_fuzzy") {
        controller_type_ = "pid";
      }
    } else {
      fuzzy_params_ = std::move(params);
      RCLCPP_INFO(get_logger(), "Loaded IT2 fuzzy parameters from '%s'", resolved_path.c_str());
    }
  }

  // Load GT2 fuzzy params if needed
  if (needs_gt2_fuzzy && !gt2_fuzzy_params_) {
    std::string resolved_path = gt2_params_file_;
    try {
      if (!fs::path(resolved_path).is_absolute()) {
        const auto share_dir = ament_index_cpp::get_package_share_directory("agent_control_pkg");
        fs::path candidate = fs::path(share_dir) / "config" / resolved_path;
        if (fs::exists(candidate)) {
          resolved_path = candidate.string();
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_DEBUG(get_logger(), "Could not resolve GT2 package share directory: %s", ex.what());
    }

    if (!fs::exists(resolved_path)) {
      resolved_path = agent_control_pkg::findConfigFilePath(gt2_params_file_);
    }

    FuzzyParams params;
    if (!agent_control_pkg::loadFuzzyParamsYAML(resolved_path, params)) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to load GT2 fuzzy parameters from '%s'. Using factory defaults.",
        resolved_path.c_str());
      // GT2 can still use factory defaults, so don't disable
    } else {
      gt2_fuzzy_params_ = std::move(params);
      RCLCPP_INFO(get_logger(), "Loaded GT2 fuzzy parameters from '%s' (%zu sets, %zu rules)",
                  resolved_path.c_str(), gt2_fuzzy_params_->sets.size(), gt2_fuzzy_params_->rules.size());
    }
  }
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

  // Helper to parse anti-windup mode string
  auto parse_anti_windup_mode = [](const std::string& mode_str) {
      if (mode_str == "none") return PIDController::AntiWindupMode::NONE;
      if (mode_str == "conditional") return PIDController::AntiWindupMode::CONDITIONAL;
      if (mode_str == "back_calculation") return PIDController::AntiWindupMode::BACK_CALCULATION;
      return PIDController::AntiWindupMode::COMBINED;  // default
    };

  auto make_pid = [&](double kp, double ki, double kd) {
      auto pid = std::make_unique<controllers::PIDAdapter>(kp, ki, kd, limits.umin, limits.umax);
      pid->enableDerivativeFilter(pid_enable_derivative_filter_, pid_derivative_filter_alpha_);
      pid->setAntiWindupMode(parse_anti_windup_mode(pid_anti_windup_mode_));
      if (pid_tracking_time_constant_ > 0.0) {
        pid->setTrackingTimeConstant(pid_tracking_time_constant_);
      }
      result.pid = pid.get();
      result.controller = std::move(pid);
    };

  auto make_fuzzy = [&]() -> std::unique_ptr<controllers::FuzzyIT2Adapter> {
      controllers::FuzzyIT2Adapter::Options opt;
      opt.umin = limits.umin;
      opt.umax = limits.umax;
      opt.wind_scalar = fuzzy_wind_scalar_;
      auto fuzzy = std::make_unique<controllers::FuzzyIT2Adapter>(opt);
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
    pid->setAntiWindupMode(parse_anti_windup_mode(pid_anti_windup_mode_));
    if (pid_tracking_time_constant_ > 0.0) {
      pid->setTrackingTimeConstant(pid_tracking_time_constant_);
    }
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

  // GT2 Fuzzy Logic System (General Type-2)
  if (type == "pid_gt2_fuzzy" || type == "gt2_fuzzy") {
    // Parse secondary shape string to enum
    auto parse_secondary_shape = [](const std::string& shape_str) {
      if (shape_str == "gaussian") return GT2FuzzyLogicSystem::SecondaryMFShape::GAUSSIAN;
      if (shape_str == "trapezoidal") return GT2FuzzyLogicSystem::SecondaryMFShape::TRAPEZOIDAL;
      if (shape_str == "uniform") return GT2FuzzyLogicSystem::SecondaryMFShape::UNIFORM;
      return GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR;  // default
    };

    controllers::FuzzyGT2Adapter::Options gt2_opt;
    gt2_opt.umin = limits.umin;
    gt2_opt.umax = limits.umax;
    gt2_opt.wind_scalar = fuzzy_wind_scalar_;
    gt2_opt.num_alpha_levels = gt2_num_alpha_levels_;
    gt2_opt.secondary_spread = gt2_secondary_spread_;
    gt2_opt.secondary_shape = parse_secondary_shape(gt2_secondary_shape_);

    if (type == "gt2_fuzzy") {
      // Pure GT2 fuzzy controller (no PID)
      auto gt2_fuzzy = std::make_unique<controllers::FuzzyGT2Adapter>(gt2_opt);
      if (gt2_fuzzy_params_) {
        gt2_fuzzy->configureFromFuzzyParams(*gt2_fuzzy_params_, fuzzy_include_wind_);
        RCLCPP_INFO(get_logger(), "Created GT2-FLS controller with custom params: alpha_levels=%d, shape=%s",
                    gt2_num_alpha_levels_, gt2_secondary_shape_.c_str());
      } else {
        gt2_fuzzy->configureDefault(fuzzy_include_wind_);
        RCLCPP_INFO(get_logger(), "Created GT2-FLS controller with factory defaults: alpha_levels=%d, shape=%s",
                    gt2_num_alpha_levels_, gt2_secondary_shape_.c_str());
      }
      result.gt2_fuzzy = gt2_fuzzy.get();
      result.controller = std::move(gt2_fuzzy);
      return result;
    }

    // Hybrid PID + GT2 Fuzzy controller
    auto pid = std::make_unique<controllers::PIDAdapter>(pid_kp_, pid_ki_, pid_kd_, limits.umin, limits.umax);
    pid->enableDerivativeFilter(pid_enable_derivative_filter_, pid_derivative_filter_alpha_);
    pid->setAntiWindupMode(parse_anti_windup_mode(pid_anti_windup_mode_));
    if (pid_tracking_time_constant_ > 0.0) {
      pid->setTrackingTimeConstant(pid_tracking_time_constant_);
    }
    auto raw_pid = pid.get();

    auto gt2_fuzzy = std::make_unique<controllers::FuzzyGT2Adapter>(gt2_opt);
    if (gt2_fuzzy_params_) {
      gt2_fuzzy->configureFromFuzzyParams(*gt2_fuzzy_params_, fuzzy_include_wind_);
    } else {
      gt2_fuzzy->configureDefault(fuzzy_include_wind_);
    }
    auto raw_gt2_fuzzy = gt2_fuzzy.get();

    // Create IT2 adapter wrapper for combined controller (GT2 adapter is IController1D compatible)
    // Note: CombinedPidFuzzyAdapter expects FuzzyIT2Adapter, but we can use a simple
    // approach by computing outputs separately and combining manually.
    // For now, we create a simple hybrid by running PID and adding GT2 correction.

    // Store pointers for diagnostics
    result.pid = raw_pid;
    result.gt2_fuzzy = raw_gt2_fuzzy;

    // Use a lambda-based combined controller
    // For simplicity, we'll compute both and combine
    class CombinedPidGT2Adapter : public controllers::IController1D {
    public:
      CombinedPidGT2Adapter(
        std::unique_ptr<controllers::PIDAdapter> pid,
        std::unique_ptr<controllers::FuzzyGT2Adapter> gt2,
        double k_pid, double k_fuzzy, double umin, double umax)
        : pid_(std::move(pid)), gt2_(std::move(gt2)),
          k_pid_(k_pid), k_fuzzy_(k_fuzzy), umin_(umin), umax_(umax) {}

      double compute(double y, double yref, double dt) override {
        double u_pid = pid_->compute(y, yref, dt);
        double u_gt2 = gt2_->compute(y, yref, dt);
        double u = k_pid_ * u_pid + k_fuzzy_ * u_gt2;
        last_pid_contrib_ = k_pid_ * u_pid;
        last_fuzzy_contrib_ = k_fuzzy_ * u_gt2;
        return std::clamp(u, umin_, umax_);
      }

      void reset() override {
        pid_->reset();
        gt2_->reset();
        last_pid_contrib_ = 0.0;
        last_fuzzy_contrib_ = 0.0;
      }

      double lastPidContribution() const { return last_pid_contrib_; }
      double lastFuzzyContribution() const { return last_fuzzy_contrib_; }

      controllers::PIDAdapter* pid() { return pid_.get(); }
      controllers::FuzzyGT2Adapter* gt2() { return gt2_.get(); }

    private:
      std::unique_ptr<controllers::PIDAdapter> pid_;
      std::unique_ptr<controllers::FuzzyGT2Adapter> gt2_;
      double k_pid_, k_fuzzy_, umin_, umax_;
      double last_pid_contrib_{0.0};
      double last_fuzzy_contrib_{0.0};
    };

    auto combined = std::make_unique<CombinedPidGT2Adapter>(
      std::move(pid), std::move(gt2_fuzzy), mix_k_pid_, mix_k_fuzzy_, limits.umin, limits.umax);

    result.pid = combined->pid();
    result.gt2_fuzzy = combined->gt2();
    result.controller = std::move(combined);

    RCLCPP_INFO(get_logger(), "Created hybrid PID+GT2-FLS controller: alpha_levels=%d, shape=%s, mix=[%.2f, %.2f], params=%s",
                gt2_num_alpha_levels_, gt2_secondary_shape_.c_str(), mix_k_pid_, mix_k_fuzzy_,
                gt2_fuzzy_params_ ? "custom" : "factory");
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
  last_target_time_ = this->now();
  target_time_initialized_ = true;
  has_target_.store(true, std::memory_order_release);
}

void AgentControllerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  *latest_odom_ = *msg;
  last_odom_time_ = this->now();
  odom_time_initialized_ = true;
  has_odom_.store(true, std::memory_order_release);
}

void AgentControllerNode::windCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  // Unified wind input handling for fuzzy + feed-forward.
  // Depending on wind_source_type_, interpret the incoming Vector3 either as
  // wind velocity [m/s] or wind force [N] and derive a scalar for the fuzzy
  // system per axis.

  const double x = msg->x;
  const double y = msg->y;

  double wind_scalar_x = 0.0;
  double wind_scalar_y = 0.0;

  if (wind_source_type_ == "force") {
    // Force input [N] → acceleration bias using mass estimate.
    const double mass = (ff_params_.mass_estimate > 1e-6) ? ff_params_.mass_estimate : 1.0;
    const double ax_bias = x / mass;
    const double ay_bias = y / mass;

    wind_env_.ax_bias = ax_bias;
    wind_env_.ay_bias = ay_bias;

    wind_scalar_x = ax_bias;
    wind_scalar_y = ay_bias;
  } else {
    // Default: treat as wind velocity [m/s] (horizontal plane).
    wind_env_.vx = x;
    wind_env_.vy = y;

    wind_scalar_x = x;
    wind_scalar_y = y;
  }

  // Scale and clamp into fuzzy universe (see fuzzy_params.yaml "wind" sets).
  const auto clamp_wind = [](double value) {
    const double limit = 10.0;
    if (!std::isfinite(value)) {
      return 0.0;
    }
    return std::clamp(value, -limit, limit);
  };

  const double scaled_x = fuzzy_wind_scalar_ * wind_scalar_x;
  const double scaled_y = fuzzy_wind_scalar_ * wind_scalar_y;
  const double fuzzy_x = clamp_wind(scaled_x);
  const double fuzzy_y = clamp_wind(scaled_y);

  last_fuzzy_wind_x_ = fuzzy_x;
  last_fuzzy_wind_y_ = fuzzy_y;

  // Feed to fuzzy controllers when wind input is enabled.
  // Check for IT2 fuzzy (fuzzy_enable_) OR GT2 fuzzy (controller type check)
  // GT2 controllers use fuzzy.enable=false but still need wind input via gt2.params_file
  const bool has_gt2_controller = (controller_type_ == "gt2_fuzzy" || controller_type_ == "pid_gt2_fuzzy");
  const bool should_feed_wind = fuzzy_include_wind_ && (fuzzy_enable_ || has_gt2_controller);

  if (should_feed_wind) {
    // IT2 expects signed wind in [-10, 10] m/s range
    if (axis_x_.fuzzy) {
      axis_x_.fuzzy->setWindScalar(fuzzy_x);
    }
    if (axis_y_.fuzzy) {
      axis_y_.fuzzy->setWindScalar(fuzzy_y);
    }

    // GT2 expects normalized wind magnitude in [0, 1] range
    // Normalize: |wind| / max_wind, where max_wind = 5 m/s for Crazyflie
    const double wind_magnitude = std::sqrt(fuzzy_x * fuzzy_x + fuzzy_y * fuzzy_y);
    const double max_wind_speed = 5.0;  // From gt2_fuzzy_params_crazyflie.yaml comment
    const double gt2_wind_normalized = std::min(1.0, wind_magnitude / max_wind_speed);

    if (axis_x_.gt2_fuzzy) {
      axis_x_.gt2_fuzzy->setWindScalar(gt2_wind_normalized);
    }
    if (axis_y_.gt2_fuzzy) {
      axis_y_.gt2_fuzzy->setWindScalar(gt2_wind_normalized);
    }
  }
}

void AgentControllerNode::windDebugTimerCallback()
{
  if (!debug_wind_input_) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Fuzzy wind scalar: x=%.3f, y=%.3f (source='%s', topic='%s')",
    last_fuzzy_wind_x_, last_fuzzy_wind_y_,
    wind_source_type_.c_str(), wind_source_topic_.c_str());
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
  rclcpp::Time odom_timestamp;
  rclcpp::Time target_timestamp;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target = *latest_target_;
    odom = *latest_odom_;
    odom_timestamp = last_odom_time_;
    target_timestamp = last_target_time_;
  }

  // Data freshness check (stale data protection)
  rclcpp::Time now = this->now();
  if (enable_stale_data_check_) {
    if (odom_time_initialized_) {
      const double odom_age = (now - odom_timestamp).seconds();
      if (odom_age > odom_stale_threshold_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Stale odometry data (age=%.3f s > threshold=%.3f s). Skipping control.",
          odom_age, odom_stale_threshold_sec_);
        // Publish zero command to stop the drone
        geometry_msgs::msg::Vector3 zero_cmd;
        zero_cmd.x = 0.0;
        zero_cmd.y = 0.0;
        zero_cmd.z = 0.0;
        cmd_pub_->publish(zero_cmd);
        return;
      }
    }

    if (target_time_initialized_) {
      const double target_age = (now - target_timestamp).seconds();
      if (target_age > target_stale_threshold_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Stale target data (age=%.3f s > threshold=%.3f s). Skipping control.",
          target_age, target_stale_threshold_sec_);
        // Publish zero command to stop the drone
        geometry_msgs::msg::Vector3 zero_cmd;
        zero_cmd.x = 0.0;
        zero_cmd.y = 0.0;
        zero_cmd.z = 0.0;
        cmd_pub_->publish(zero_cmd);
        return;
      }
    }
  }

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

  // Get current velocity for feed-forward prediction
  const double vx = odom.twist.twist.linear.x;
  const double vy = odom.twist.twist.linear.y;

  // Step 1: Compute feedback control (PID/Fuzzy)
  double ax = axis_x_.controller->compute(current_x, target_x, dt);
  double ay = axis_y_.controller->compute(current_y, target_y, dt);

  // Step 2: Add feed-forward compensation (drag + wind cancellation)
  if (ff_params_.enable_drag_ff || ff_params_.enable_wind_ff) {
    double ax_ff = 0.0;
    double ay_ff = 0.0;
    core::DronePhysicsCore::computeFeedForwardCompensation(
      vx, vy, wind_env_, ff_params_, ax_ff, ay_ff
    );
    ax += ax_ff;
    ay += ay_ff;
  }

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

#ifdef CRAZYFLIE_SUPPORT
  // Dual-output: Also publish to Crazyflie if enabled
  if (crazyflie_enable_ && cf_cmd_pub_ && accel_to_attitude_) {
    // Convert acceleration to attitude commands
    // This follows the thesis approach: our controller outputs acceleration,
    // which is converted to roll/pitch/thrust for the Crazyflie's attitude controller
    auto att_cmd = accel_to_attitude_->convert(ax, ay, 0.0);  // az = 0 for hover

    crazyflie_interfaces::msg::FullState cf_msg;
    cf_msg.header.stamp = this->now();
    cf_msg.header.frame_id = "world";

    // Target position (where we want to be)
    cf_msg.pose.position.x = target_x;
    cf_msg.pose.position.y = target_y;
    cf_msg.pose.position.z = odom.pose.pose.position.z;

    // Convert roll/pitch to quaternion for desired orientation
    // Note: Using simplified Euler to quaternion (yaw = current yaw)
    double current_yaw = 0.0;  // Could extract from odom quaternion
    double cy = std::cos(current_yaw * 0.5);
    double sy = std::sin(current_yaw * 0.5);
    double cp = std::cos(att_cmd.pitch * 0.5);
    double sp = std::sin(att_cmd.pitch * 0.5);
    double cr = std::cos(att_cmd.roll * 0.5);
    double sr = std::sin(att_cmd.roll * 0.5);

    cf_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    cf_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    cf_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    cf_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy;

    // Target velocity (could be computed from trajectory)
    cf_msg.twist.linear.x = 0.0;
    cf_msg.twist.linear.y = 0.0;
    cf_msg.twist.linear.z = 0.0;

    // Acceleration command (from our controller)
    cf_msg.acc.x = ax;
    cf_msg.acc.y = ay;
    cf_msg.acc.z = 0.0;

    cf_cmd_pub_->publish(cf_msg);

    RCLCPP_DEBUG(get_logger(),
      "CF cmd: roll=%.3f, pitch=%.3f, thrust=%.3f (ax=%.3f, ay=%.3f)",
      att_cmd.roll, att_cmd.pitch, att_cmd.thrust, ax, ay);
  }
#endif

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

void AgentControllerNode::publishControllerParams()
{
  if (!params_pub_) {
    return;
  }

  auto msg = my_custom_interfaces_pkg::msg::ControllerParams();

  // Controller type
  msg.controller_type = controller_type_;

  // PID parameters
  msg.pid_kp = pid_kp_;
  msg.pid_ki = pid_ki_;
  msg.pid_kd = pid_kd_;

  // Fuzzy parameters
  msg.fuzzy_enable = fuzzy_enable_;
  msg.fuzzy_wind_scalar = fuzzy_wind_scalar_;

  // Hybrid mixing
  msg.mix_k_pid = mix_k_pid_;
  msg.mix_k_fuzzy = mix_k_fuzzy_;

  // Feed-forward parameters
  msg.feedforward_enable_drag = ff_params_.enable_drag_ff;
  msg.feedforward_enable_wind = ff_params_.enable_wind_ff;
  msg.feedforward_k_drag = ff_params_.k_drag;
  msg.feedforward_k_wind = ff_params_.k_wind;

  // Output limits
  msg.output_limit_x_min = axis_limits_x_.umin;
  msg.output_limit_x_max = axis_limits_x_.umax;
  msg.output_limit_y_min = axis_limits_y_.umin;
  msg.output_limit_y_max = axis_limits_y_.umax;

  // Control frequency
  msg.control_frequency_hz = (loop_period_ > 0.0) ? (1.0 / loop_period_) : 0.0;
  msg.dt = configured_dt_;

  params_pub_->publish(msg);
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
