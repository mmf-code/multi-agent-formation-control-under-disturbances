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

#include "agent_control_pkg/config_reader.hpp"
#include "agent_control_pkg/controllers/combined_pid_fuzzy_adapter.hpp"
#include "agent_control_pkg/controllers/fuzzy_gt2_adapter.hpp"
#include "agent_control_pkg/controllers/pid_adapter.hpp"

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
    controllers::FuzzyGT2Adapter * fuzzy{nullptr};
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
  void controlLoop();
  void publishDiagnostics();
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
  std::string fuzzy_params_file_{"fuzzy_params.yaml"};
  bool publish_diagnostics_{true};

  double pid_kp_{0.6};
  double pid_ki_{0.0};
  double pid_kd_{0.0};
  bool pid_enable_derivative_filter_{true};
  double pid_derivative_filter_alpha_{0.1};

  std::optional<agent_control_pkg::FuzzyParams> fuzzy_params_;

  AxisController axis_x_;
  AxisController axis_y_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped::SharedPtr latest_target_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::mutex data_mutex_;

  rclcpp::Time last_control_time_;
  bool first_control_cycle_{true};
  std::atomic<bool> has_target_{false};
  std::atomic<bool> has_odom_{false};
};

}  // namespace agent_control_pkg

#endif  // AGENT_CONTROL_PKG__AGENT_CONTROLLER_NODE_HPP_
