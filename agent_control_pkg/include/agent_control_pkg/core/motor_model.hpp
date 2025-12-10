/**
 * @file motor_model.hpp
 * @brief Crazyflie motor dynamics model based on sim_cf2 parameters
 *
 * Motor Model (from sim_cf2/CrazyflieTHI):
 *   omega_dot = (omega_cmd - omega) / tau
 *   tau = tau_up (if omega_cmd > omega) else tau_down
 *
 * Thrust Model:
 *   T = k_thrust * omega^2
 *
 * Yaw Torque Model:
 *   tau_yaw = direction * k_moment * T
 *
 * X-Config Layout:
 *      FL(CCW)    FR(CW)
 *         \      /
 *          Center
 *         /      \
 *      BL(CW)    BR(CCW)
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

namespace agent_control_pkg {
namespace core {

/**
 * @brief Motor configuration parameters (sim_cf2 defaults)
 */
struct MotorParams {
  // Time constants (asymmetric)
  double tau_up = 0.0125;      ///< [s] Motor spin-up time constant
  double tau_down = 0.025;     ///< [s] Motor spin-down time constant

  // Speed limits
  double omega_max = 3052.0;   ///< [rad/s] Maximum angular velocity
  double omega_min = 0.0;      ///< [rad/s] Minimum angular velocity (0 or idle)

  // Thrust coefficient: T = k_thrust * omega^2
  double k_thrust = 1.2819e-8; ///< [N/(rad/s)^2] Thrust coefficient

  // Moment coefficient: tau_yaw = dir * k_moment * T
  double k_moment = 0.005965;  ///< [dimensionless] Moment-to-thrust ratio

  // Physical layout (X-config)
  double arm_length = 0.0325;  ///< [m] Distance from CoM to motor (diagonal/2)

  /**
   * @brief Factory method for sim_cf2 Crazyflie parameters
   */
  static MotorParams sim_cf2() {
    MotorParams p;
    p.tau_up = 0.0125;
    p.tau_down = 0.025;
    p.omega_max = 3052.0;
    p.omega_min = 0.0;
    p.k_thrust = 1.2819e-8;
    p.k_moment = 0.005965;
    p.arm_length = 0.0325;
    return p;
  }

  /**
   * @brief Compute hover omega for given mass
   * omega_hover = sqrt(m * g / (4 * k_thrust))
   */
  double hoverOmega(double mass, double gravity = 9.81) const {
    return std::sqrt(mass * gravity / (4.0 * k_thrust));
  }

  /**
   * @brief Compute maximum total thrust
   * T_max = 4 * k_thrust * omega_max^2
   */
  double maxTotalThrust() const {
    return 4.0 * k_thrust * omega_max * omega_max;
  }
};

/**
 * @brief Motor state for a single motor
 */
struct MotorState {
  double omega = 0.0;      ///< [rad/s] Current angular velocity
  double omega_cmd = 0.0;  ///< [rad/s] Commanded angular velocity
  double thrust = 0.0;     ///< [N] Current thrust force
  double torque = 0.0;     ///< [Nm] Current yaw torque contribution
};

/**
 * @brief Rotor direction enum
 */
enum class RotorDirection : int {
  CW = -1,   ///< Clockwise (negative yaw torque)
  CCW = +1   ///< Counter-clockwise (positive yaw torque)
};

/**
 * @brief Complete motor model for 4-rotor quadcopter (X-config)
 *
 * This class implements the motor dynamics from sim_cf2:
 * - First-order lag with asymmetric time constants
 * - Quadratic thrust model
 * - Yaw torque from motor reaction
 */
class CrazyflieMotorModel {
public:
  // Rotor indices (X-config)
  static constexpr int FR = 0;  ///< Front-Right
  static constexpr int FL = 1;  ///< Front-Left
  static constexpr int BL = 2;  ///< Back-Left
  static constexpr int BR = 3;  ///< Back-Right

  /**
   * @brief Construct motor model with given parameters
   */
  explicit CrazyflieMotorModel(const MotorParams& params = MotorParams::sim_cf2())
    : params_(params)
  {
    // X-config directions (diagonal pairs spin same direction for yaw balance)
    directions_ = {
      RotorDirection::CW,   // FR - clockwise
      RotorDirection::CCW,  // FL - counter-clockwise
      RotorDirection::CW,   // BL - clockwise
      RotorDirection::CCW   // BR - counter-clockwise
    };

    // X-config positions relative to CoM [x, y, z]
    // Rotated 45 degrees from body axes
    const double L = params_.arm_length;
    positions_ = {{
      {{ L,  -L, 0.0}},  // FR: +X, -Y
      {{ L,   L, 0.0}},  // FL: +X, +Y
      {{-L,   L, 0.0}},  // BL: -X, +Y
      {{-L,  -L, 0.0}}   // BR: -X, -Y
    }};
  }

  /**
   * @brief Update motor dynamics for one timestep
   * @param dt Time step [s]
   */
  void update(double dt) {
    for (int i = 0; i < 4; ++i) {
      updateSingleMotor(i, dt);
    }
  }

  /**
   * @brief Set commanded angular velocity for motor i
   */
  void setOmegaCmd(int i, double omega_cmd) {
    if (i >= 0 && i < 4) {
      motors_[i].omega_cmd = std::clamp(omega_cmd, params_.omega_min, params_.omega_max);
    }
  }

  /**
   * @brief Set all motor commands at once
   */
  void setAllOmegaCmd(const std::array<double, 4>& omega_cmds) {
    for (int i = 0; i < 4; ++i) {
      setOmegaCmd(i, omega_cmds[i]);
    }
  }

  /**
   * @brief Get current motor state
   */
  const MotorState& getMotor(int i) const {
    return motors_[i];
  }

  /**
   * @brief Get all motor states
   */
  const std::array<MotorState, 4>& getMotors() const {
    return motors_;
  }

  /**
   * @brief Get total thrust (sum of all motors)
   */
  double getTotalThrust() const {
    double total = 0.0;
    for (const auto& m : motors_) {
      total += m.thrust;
    }
    return total;
  }

  /**
   * @brief Get total yaw torque (sum of motor torques)
   * Positive = CCW rotation of body
   */
  double getTotalYawTorque() const {
    double total = 0.0;
    for (int i = 0; i < 4; ++i) {
      total += motors_[i].torque;
    }
    return total;
  }

  /**
   * @brief Get roll torque from thrust differential
   * Roll = (FL + BL) - (FR + BR) thrust differential * arm
   * Positive roll = right wing down
   */
  double getRollTorque() const {
    double left_thrust = motors_[FL].thrust + motors_[BL].thrust;
    double right_thrust = motors_[FR].thrust + motors_[BR].thrust;
    return (left_thrust - right_thrust) * params_.arm_length;
  }

  /**
   * @brief Get pitch torque from thrust differential
   * Pitch = (FL + FR) - (BL + BR) thrust differential * arm
   * Positive pitch = nose up
   */
  double getPitchTorque() const {
    double front_thrust = motors_[FL].thrust + motors_[FR].thrust;
    double back_thrust = motors_[BL].thrust + motors_[BR].thrust;
    return (front_thrust - back_thrust) * params_.arm_length;
  }

  /**
   * @brief Get rotor position relative to CoM [x, y, z]
   */
  const std::array<double, 3>& getRotorPosition(int i) const {
    return positions_[i];
  }

  /**
   * @brief Get rotor direction (+1 CCW, -1 CW)
   */
  int getRotorDirection(int i) const {
    return static_cast<int>(directions_[i]);
  }

  /**
   * @brief Get parameters
   */
  const MotorParams& getParams() const { return params_; }

  /**
   * @brief Reset all motors to zero
   */
  void reset() {
    for (auto& m : motors_) {
      m.omega = 0.0;
      m.omega_cmd = 0.0;
      m.thrust = 0.0;
      m.torque = 0.0;
    }
  }

  /**
   * @brief Set all motors to hover omega for given mass
   */
  void setHover(double mass, double gravity = 9.81) {
    double omega_hover = params_.hoverOmega(mass, gravity);
    for (int i = 0; i < 4; ++i) {
      motors_[i].omega = omega_hover;
      motors_[i].omega_cmd = omega_hover;
      motors_[i].thrust = params_.k_thrust * omega_hover * omega_hover;
      motors_[i].torque = static_cast<int>(directions_[i]) * params_.k_moment * motors_[i].thrust;
    }
  }

private:
  /**
   * @brief Update single motor dynamics
   */
  void updateSingleMotor(int i, double dt) {
    auto& m = motors_[i];

    // Asymmetric first-order dynamics
    double tau = (m.omega_cmd > m.omega) ? params_.tau_up : params_.tau_down;

    // First-order filter: omega_dot = (omega_cmd - omega) / tau
    if (tau > 1e-9) {
      double alpha = dt / tau;
      m.omega += alpha * (m.omega_cmd - m.omega);
    } else {
      m.omega = m.omega_cmd;
    }

    // Clamp to valid range
    m.omega = std::clamp(m.omega, params_.omega_min, params_.omega_max);

    // Compute thrust: T = k_thrust * omega^2
    m.thrust = params_.k_thrust * m.omega * m.omega;

    // Compute yaw torque: tau = dir * k_moment * T
    int dir = static_cast<int>(directions_[i]);
    m.torque = dir * params_.k_moment * m.thrust;
  }

  MotorParams params_;
  std::array<MotorState, 4> motors_{};
  std::array<RotorDirection, 4> directions_{};
  std::array<std::array<double, 3>, 4> positions_{};
};

}  // namespace core
}  // namespace agent_control_pkg
