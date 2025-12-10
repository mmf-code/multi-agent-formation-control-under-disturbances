/**
 * @file motor_mixer.hpp
 * @brief Control allocation / mixing for X-config quadrotor
 *
 * Converts body-frame commands to individual motor speeds:
 *   [omega_fr, omega_fl, omega_bl, omega_br] = f(thrust, roll, pitch, yaw)
 *
 * X-Config Mixing Matrix:
 *   Motor layout (top view, + is front):
 *      FL(CCW)    FR(CW)
 *         \      /
 *          Center
 *         /      \
 *      BL(CW)    BR(CCW)
 *
 * Mixing equations (thrust to motor forces):
 *   T_total = T_fr + T_fl + T_bl + T_br
 *   tau_roll = L * (T_fl + T_bl - T_fr - T_br)
 *   tau_pitch = L * (T_fl + T_fr - T_bl - T_br)
 *   tau_yaw = k_m * (-T_fr + T_fl - T_bl + T_br)
 *
 * Reference: https://github.com/CrazyflieTHI/sim_cf2
 *
 * @author Multi-Agent Formation Control Team
 * @date 2025
 */

#pragma once

#include <array>
#include <cmath>
#include <algorithm>
#include "motor_model.hpp"

namespace agent_control_pkg {
namespace core {

/**
 * @brief Mixer configuration
 */
struct MixerConfig {
  double k_thrust = 1.2819e-8;  ///< [N/(rad/s)^2] Thrust coefficient
  double omega_max = 3052.0;    ///< [rad/s] Maximum motor speed
  double omega_min = 0.0;       ///< [rad/s] Minimum motor speed
  double arm_length = 0.0325;   ///< [m] Motor arm length
  double k_moment = 0.005965;   ///< Moment coefficient

  /**
   * @brief Create config from MotorParams
   */
  static MixerConfig fromMotorParams(const MotorParams& p) {
    MixerConfig c;
    c.k_thrust = p.k_thrust;
    c.omega_max = p.omega_max;
    c.omega_min = p.omega_min;
    c.arm_length = p.arm_length;
    c.k_moment = p.k_moment;
    return c;
  }
};

/**
 * @brief Body-frame control commands
 */
struct BodyCommands {
  double thrust = 0.0;       ///< [N] Total collective thrust
  double roll_torque = 0.0;  ///< [Nm] Roll moment command
  double pitch_torque = 0.0; ///< [Nm] Pitch moment command
  double yaw_torque = 0.0;   ///< [Nm] Yaw moment command
};

/**
 * @brief X-configuration mixer for quadrotor
 *
 * Converts body-frame commands (thrust, roll, pitch, yaw) to
 * individual motor angular velocities.
 */
class XConfigMixer {
public:
  /**
   * @brief Construct mixer with given configuration
   */
  explicit XConfigMixer(const MixerConfig& config = MixerConfig())
    : config_(config)
  {
  }

  /**
   * @brief Convert body commands to motor angular velocities
   * @param cmd Body-frame commands
   * @return Array of omega commands [FR, FL, BL, BR]
   *
   * Inverse mixing equations for X-config:
   *   T_fr = T/4 - roll/(4L) + pitch/(4L) - yaw/(4*km)
   *   T_fl = T/4 + roll/(4L) + pitch/(4L) + yaw/(4*km)
   *   T_bl = T/4 + roll/(4L) - pitch/(4L) - yaw/(4*km)
   *   T_br = T/4 - roll/(4L) - pitch/(4L) + yaw/(4*km)
   */
  std::array<double, 4> mix(const BodyCommands& cmd) const {
    const double L = config_.arm_length;
    const double km = config_.k_moment;
    const double kt = config_.k_thrust;

    // Base thrust per motor
    double T_base = cmd.thrust / 4.0;

    // Contributions from roll/pitch/yaw
    double roll_contrib = (L > 1e-9) ? cmd.roll_torque / (4.0 * L) : 0.0;
    double pitch_contrib = (L > 1e-9) ? cmd.pitch_torque / (4.0 * L) : 0.0;
    double yaw_contrib = (km > 1e-9) ? cmd.yaw_torque / (4.0 * km) : 0.0;

    // Compute individual motor thrusts
    std::array<double, 4> thrusts;
    thrusts[CrazyflieMotorModel::FR] = T_base - roll_contrib + pitch_contrib - yaw_contrib;
    thrusts[CrazyflieMotorModel::FL] = T_base + roll_contrib + pitch_contrib + yaw_contrib;
    thrusts[CrazyflieMotorModel::BL] = T_base + roll_contrib - pitch_contrib - yaw_contrib;
    thrusts[CrazyflieMotorModel::BR] = T_base - roll_contrib - pitch_contrib + yaw_contrib;

    // Convert thrust to omega: omega = sqrt(T / kt)
    std::array<double, 4> omegas;
    for (int i = 0; i < 4; ++i) {
      double T = std::max(thrusts[i], 0.0);  // No negative thrust
      double omega = (kt > 1e-15) ? std::sqrt(T / kt) : 0.0;
      omegas[i] = std::clamp(omega, config_.omega_min, config_.omega_max);
    }

    return omegas;
  }

  /**
   * @brief Simple collective-only mixing
   * All motors get same omega for collective thrust (no attitude control)
   * @param thrust_total Total thrust [N]
   * @return Array of equal omega commands
   */
  std::array<double, 4> mixCollectiveOnly(double thrust_total) const {
    double T_per_motor = thrust_total / 4.0;
    T_per_motor = std::max(T_per_motor, 0.0);
    double omega = (config_.k_thrust > 1e-15)
      ? std::sqrt(T_per_motor / config_.k_thrust)
      : 0.0;
    omega = std::clamp(omega, config_.omega_min, config_.omega_max);
    return {omega, omega, omega, omega};
  }

  /**
   * @brief Get mixer configuration
   */
  const MixerConfig& getConfig() const { return config_; }

private:
  MixerConfig config_;
};

/**
 * @brief Convert acceleration command to thrust (simple model)
 *
 * For initial testing: converts Z acceleration to collective thrust
 * without attitude conversion.
 */
class AccelToThrustSimple {
public:
  /**
   * @brief Construct converter with mass and gravity
   */
  explicit AccelToThrustSimple(double mass = 0.027, double gravity = 9.81)
    : mass_(mass), gravity_(gravity) {}

  /**
   * @brief Convert Z acceleration command to total thrust
   * T = m * (g + az_cmd)
   * @param az_cmd Commanded vertical acceleration [m/s^2] (positive = up)
   * @return Total thrust [N]
   */
  double accelToThrust(double az_cmd) const {
    double required_accel = gravity_ + az_cmd;  // Include gravity compensation
    required_accel = std::max(required_accel, 0.0);  // Can't push down
    return mass_ * required_accel;
  }

  /**
   * @brief Get hover thrust (for XY-only control)
   * @return Hover thrust [N]
   */
  double hoverThrust() const {
    return mass_ * gravity_;
  }

  /**
   * @brief Get mass
   */
  double getMass() const { return mass_; }

  /**
   * @brief Get gravity
   */
  double getGravity() const { return gravity_; }

  /**
   * @brief Set mass (for dynamic updates)
   */
  void setMass(double mass) { mass_ = mass; }

private:
  double mass_;
  double gravity_;
};

}  // namespace core
}  // namespace agent_control_pkg
