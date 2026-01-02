#ifndef AGENT_CONTROL_PKG__AGENT_CONTROLLER_NODE_HPP_
#define AGENT_CONTROL_PKG__AGENT_CONTROLLER_NODE_HPP_

#include <atomic>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <my_custom_interfaces_pkg/msg/controller_params.hpp>

#ifdef CRAZYFLIE_SUPPORT
#include <crazyflie_interfaces/msg/full_state.hpp>
#endif

#include "agent_control_pkg/config_reader.hpp"
#include "agent_control_pkg/controllers/combined_pid_fuzzy_adapter.hpp"
#include "agent_control_pkg/controllers/fuzzy_it2_adapter.hpp"
#include "agent_control_pkg/controllers/fuzzy_gt2_adapter.hpp"
#include "agent_control_pkg/controllers/pid_adapter.hpp"
#include "agent_control_pkg/core/physics_types.hpp"
#include "agent_control_pkg/core/drone_physics_core.hpp"
#include "agent_control_pkg/core/accel_to_attitude.hpp"

namespace agent_control_pkg
{

class AgentControllerNode : public rclcpp::Node
{
public:
  explicit AgentControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct AxisController
  {
    std::unique_ptr<controllers::IController1D> controller;
    controllers::PIDAdapter * pid{nullptr};
    controllers::FuzzyIT2Adapter * fuzzy{nullptr};
    controllers::FuzzyGT2Adapter * gt2_fuzzy{nullptr};  // GT2 fuzzy controller pointer
    controllers::CombinedPidFuzzyAdapter * combined{nullptr};
    double last_total_output{0.0};
    double last_pid_contribution{0.0};
    double last_fuzzy_contribution{0.0};
  };

  struct AxisLimits
  {
    double umin{-10.0};
    double umax{10.0};
  };

  void declareParameters();
  void loadParameters();
  void configureControllers();
  AxisController createAxisController(const std::string & axis_name, const AxisLimits & limits);
  void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void windCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void windDebugTimerCallback();
  void controlLoop();
  void publishDiagnostics();
  void publishControllerParams();
  void ensureFuzzyParamsLoaded();

  AxisLimits axis_limits_x_;
  AxisLimits axis_limits_y_;

  std::string controller_type_{"pid"};
  double configured_dt_{0.005};
  double loop_period_{0.005};
  double mix_k_pid_{1.0};
  double mix_k_fuzzy_{1.0};
  bool fuzzy_enable_{false};
  bool fuzzy_include_wind_{false};
  double fuzzy_wind_scalar_{0.0};
   // Wind input configuration for fuzzy + feed-forward
  std::string wind_source_topic_{"/wind/velocity"};
  std::string wind_source_type_{"velocity"};  // "velocity" or "force"
  bool debug_wind_input_{false};
  double last_fuzzy_wind_x_{0.0};
  double last_fuzzy_wind_y_{0.0};
  std::string fuzzy_params_file_{"fuzzy_params.yaml"};
  bool publish_diagnostics_{true};

  // GT2 Fuzzy specific parameters
  int gt2_num_alpha_levels_{5};  // Number of alpha-planes for GT2
  std::string gt2_secondary_shape_{"triangular"};  // "triangular", "gaussian", "trapezoidal", "uniform"

  double pid_kp_{0.6};
  double pid_ki_{0.0};
  double pid_kd_{0.0};
  bool pid_enable_derivative_filter_{true};
  double pid_derivative_filter_alpha_{0.1};
  std::string pid_anti_windup_mode_{"combined"};
  double pid_tracking_time_constant_{0.0};

  // Data freshness (stale data protection)
  bool enable_stale_data_check_{true};
  double odom_stale_threshold_sec_{0.5};   // Odometry data older than this is stale
  double target_stale_threshold_sec_{2.0}; // Target pose older than this is stale

  std::optional<agent_control_pkg::FuzzyParams> fuzzy_params_;

  // Feed-forward control parameters
  core::FeedForwardParams ff_params_;
  core::WindEnvironment wind_env_;

  AxisController axis_x_;
  AxisController axis_y_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<my_custom_interfaces_pkg::msg::ControllerParams>::SharedPtr params_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr params_timer_;
  rclcpp::TimerBase::SharedPtr wind_debug_timer_;

#ifdef CRAZYFLIE_SUPPORT
  // Crazyflie dual-output support
  bool crazyflie_enable_{false};
  std::string crazyflie_cmd_topic_{"cmd_full_state"};
  rclcpp::Publisher<crazyflie_interfaces::msg::FullState>::SharedPtr cf_cmd_pub_;

  // Acceleration to attitude converter for Crazyflie
  core::AccelToAttitudeConfig cf_attitude_config_;
  std::unique_ptr<core::AccelToAttitude> accel_to_attitude_;
#endif

  geometry_msgs::msg::PoseStamped::SharedPtr latest_target_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::mutex data_mutex_;

  rclcpp::Time last_control_time_;
  bool first_control_cycle_{true};
  std::atomic<bool> has_target_{false};
  std::atomic<bool> has_odom_{false};

  // Timestamps for data freshness checking
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_target_time_;
  bool odom_time_initialized_{false};
  bool target_time_initialized_{false};
};

}  // namespace agent_control_pkg

#endif  // AGENT_CONTROL_PKG__AGENT_CONTROLLER_NODE_HPP_
