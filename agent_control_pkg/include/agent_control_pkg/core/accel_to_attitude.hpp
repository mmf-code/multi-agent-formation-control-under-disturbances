/**
 * @file accel_to_attitude.hpp
 * @brief Converts XY acceleration commands to roll/pitch/thrust for Crazyflie
 *
 * This converter implements the standard quadrotor attitude-to-acceleration
 * relationship for near-hover flight:
 *
 * Physics (small angle approximation):
 *   ax = g * tan(theta) ≈ g * theta   →  theta = atan(ax / g)
 *   ay = -g * tan(phi) ≈ -g * phi     →  phi = atan(-ay / g)
 *   az = (thrust / mass) - g          →  thrust = mass * (az + g)
 *
 * Where:
 *   - phi (roll): positive roll → positive Y acceleration
 *   - theta (pitch): positive pitch → positive X acceleration
 *   - g ≈ 9.81 m/s²
 *
 * Coordinate frames:
 *   - NED (North-East-Down): Crazyflie default
 *   - ENU (East-North-Up): ROS2 default
 *   - This converter assumes ENU input and outputs ENU-compatible angles
 *
 * @note For the Crazyflie, attitude commands go through its internal
 *       attitude controller (Mellinger or similar), so we're not bypassing
 *       the flight controller - we're commanding attitude setpoints.
 *
 * @author Control Systems Research Team
 * @date 2024-12
 */

#ifndef AGENT_CONTROL_PKG__CORE__ACCEL_TO_ATTITUDE_HPP_
#define AGENT_CONTROL_PKG__CORE__ACCEL_TO_ATTITUDE_HPP_

#include <cmath>
#include <algorithm>

namespace agent_control_pkg::core
{

/**
 * @brief Output structure for attitude/thrust commands
 */
struct AttitudeCommand
{
  double roll{0.0};      // [rad] Roll angle (phi)
  double pitch{0.0};     // [rad] Pitch angle (theta)
  double yaw{0.0};       // [rad] Yaw angle (psi) - typically pass-through
  double thrust{0.0};    // [N] Total thrust force OR [0-1] normalized
  double thrust_normalized{0.0};  // [0-1] Normalized thrust (for Crazyflie PWM)
};

/**
 * @brief Configuration parameters for AccelToAttitude converter
 */
struct AccelToAttitudeConfig
{
  // Physical parameters
  double gravity{9.81};           // [m/s²] Gravitational acceleration
  double mass{0.027};             // [kg] Vehicle mass (Crazyflie ≈ 27g)

  // Thrust parameters
  double hover_thrust{0.265};     // [N] Thrust at hover ≈ mass * g
  double max_thrust{0.6};         // [N] Maximum thrust (Crazyflie ≈ 60g)
  double min_thrust{0.0};         // [N] Minimum thrust

  // Attitude limits (safety bounds)
  double max_roll_rad{0.5236};    // [rad] Max roll (30°)
  double max_pitch_rad{0.5236};   // [rad] Max pitch (30°)

  // Crazyflie-specific normalization
  // Crazyflie uses 16-bit PWM (0-65535) but often normalized to 0-1
  bool normalize_thrust{true};    // Output normalized thrust [0-1]
};

/**
 * @class AccelToAttitude
 * @brief Converts desired acceleration to attitude commands for quadrotors
 *
 * This class provides the mathematical conversion from desired XY acceleration
 * to roll/pitch angles, suitable for commanding attitude-based quadrotor
 * controllers like those on the Crazyflie.
 */
class AccelToAttitude
{
public:
  explicit AccelToAttitude(const AccelToAttitudeConfig& config = AccelToAttitudeConfig{})
    : config_(config)
  {
    // Precompute hover thrust if not explicitly set
    if (config_.hover_thrust <= 0.0) {
      config_.hover_thrust = config_.mass * config_.gravity;
    }
  }

  /**
   * @brief Convert acceleration command to attitude command
   *
   * @param ax Desired X acceleration [m/s²] (positive = forward)
   * @param ay Desired Y acceleration [m/s²] (positive = left)
   * @param az Desired Z acceleration [m/s²] (positive = up, 0 = hover)
   * @param yaw_cmd Desired yaw angle [rad] (pass-through, typically from path planner)
   * @return AttitudeCommand with roll, pitch, yaw, and thrust
   */
  AttitudeCommand convert(double ax, double ay, double az, double yaw_cmd = 0.0) const
  {
    AttitudeCommand cmd;
    cmd.yaw = yaw_cmd;

    // Clamp input accelerations for safety
    const double max_horiz_accel = config_.gravity * std::tan(config_.max_pitch_rad);
    ax = std::clamp(ax, -max_horiz_accel, max_horiz_accel);
    ay = std::clamp(ay, -max_horiz_accel, max_horiz_accel);

    // =========================================================================
    // PITCH (theta) from ax
    // For ENU frame: positive pitch → nose up → positive X acceleration
    // ax = g * tan(theta)  →  theta = atan(ax / g)
    // =========================================================================
    cmd.pitch = std::atan2(ax, config_.gravity);
    cmd.pitch = std::clamp(cmd.pitch, -config_.max_pitch_rad, config_.max_pitch_rad);

    // =========================================================================
    // ROLL (phi) from ay
    // For ENU frame: positive roll → right wing down → NEGATIVE Y acceleration
    // ay = -g * tan(phi)  →  phi = atan(-ay / g)
    // =========================================================================
    cmd.roll = std::atan2(-ay, config_.gravity);
    cmd.roll = std::clamp(cmd.roll, -config_.max_roll_rad, config_.max_roll_rad);

    // =========================================================================
    // THRUST from az
    // Total thrust needed to achieve desired vertical acceleration
    // az = (thrust / mass) - g  →  thrust = mass * (az + g)
    //
    // Account for the fact that tilted thrust reduces vertical component:
    // thrust_vertical = thrust * cos(roll) * cos(pitch)
    // =========================================================================
    const double cos_roll = std::cos(cmd.roll);
    const double cos_pitch = std::cos(cmd.pitch);
    const double tilt_factor = cos_roll * cos_pitch;

    // Ensure tilt_factor doesn't go too low (extreme angles)
    const double safe_tilt_factor = std::max(tilt_factor, 0.5);

    // Required thrust accounting for tilt
    const double thrust_needed = config_.mass * (az + config_.gravity) / safe_tilt_factor;
    cmd.thrust = std::clamp(thrust_needed, config_.min_thrust, config_.max_thrust);

    // Normalize thrust for Crazyflie [0-1]
    if (config_.normalize_thrust && config_.max_thrust > 0.0) {
      cmd.thrust_normalized = cmd.thrust / config_.max_thrust;
      cmd.thrust_normalized = std::clamp(cmd.thrust_normalized, 0.0, 1.0);
    }

    return cmd;
  }

  /**
   * @brief Update configuration (e.g., for parameter changes)
   */
  void setConfig(const AccelToAttitudeConfig& config)
  {
    config_ = config;
    if (config_.hover_thrust <= 0.0) {
      config_.hover_thrust = config_.mass * config_.gravity;
    }
  }

  const AccelToAttitudeConfig& getConfig() const { return config_; }

private:
  AccelToAttitudeConfig config_;
};

}  // namespace agent_control_pkg::core

#endif  // AGENT_CONTROL_PKG__CORE__ACCEL_TO_ATTITUDE_HPP_
