/**
 * @file physics_types.hpp
 * @brief Shared physics data structures for drone dynamics simulation
 *
 * This header-only library defines common data structures used by both
 * standalone C++ simulations and ROS2/Gazebo integration. It provides
 * a single source of truth for physics parameters and state variables.
 *
 * Design Philosophy:
 *   - Header-only: No .cpp file needed, all inline
 *   - Zero ROS dependencies: Pure C++ for maximum reusability
 *   - Shared by drone_dynamics_2d.cpp and simple_drone_plugin.cpp
 *
 * @author Multi-Agent Formation Control Team
 * @date 2025
 */

#pragma once

#include <cmath>
#include <algorithm>

namespace agent_control_pkg {
namespace core {

/**
 * @struct PhysicsParams
 * @brief Complete set of 2D drone physics parameters
 *
 * These parameters define the physical characteristics and dynamics model
 * of the quadrotor. All parameters match the standalone simulation for consistency.
 */
struct PhysicsParams {
  // ===== Inertial Properties =====
  double mass = 1.5;  ///< Drone mass [kg]

  // ===== Actuator Dynamics (First-Order Lag) =====
  /// Symmetric time constant [s] (used if asymmetric not set)
  double actuator_tau = 0.1;

  /// Rising edge time constant [s] (acceleration increase)
  /// If <= 0, falls back to actuator_tau
  double actuator_tau_up = 0.1;

  /// Falling edge time constant [s] (acceleration decrease)
  /// If <= 0, falls back to actuator_tau
  /// Typically slower than tau_up due to motor/ESC physics
  double actuator_tau_down = 0.15;

  /// Maximum acceleration command [m/s²] (saturation limit)
  double max_accel = 10.0;

  // ===== Drag Model (Linear → Quadratic Blend) =====
  /// Linear drag coefficient [N/(m/s)] or equivalently [kg/s]
  /// Force = -cd_lin * v_rel
  double drag_coeff_lin = 0.12;

  /// Quadratic drag coefficient [N/(m/s)²] or [kg/m]
  /// Force = -cd_quad * |v_rel| * v_rel
  double drag_coeff_quad = 0.05;

  /// Speed threshold [m/s] for drag model transition
  /// Below threshold: linear drag dominates
  /// Above threshold: quadratic drag dominates
  /// Smooth blend in between using sigmoid-like function
  double drag_speed_threshold = 1.0;

  // ===== Diagnostic Flags =====
  /// Enable detailed physics logging (for debugging)
  bool enable_diagnostics = false;
};

/**
 * @struct WindEnvironment
 * @brief Ambient wind and disturbance forces
 *
 * Two disturbance models:
 *   1. Velocity-based: Wind creates relative velocity → affects drag
 *   2. Acceleration-based: Constant force bias (e.g., unmodeled forces)
 */
struct WindEnvironment {
  // ===== Velocity-Based Wind =====
  double vx = 0.0;  ///< Wind velocity X [m/s]
  double vy = 0.0;  ///< Wind velocity Y [m/s]

  // ===== Acceleration Bias (Constant Disturbance) =====
  /// Constant acceleration disturbance X [m/s²]
  /// Models unmodeled forces: F_unknown = m * ax_bias
  double ax_bias = 0.0;

  /// Constant acceleration disturbance Y [m/s²]
  double ay_bias = 0.0;
};

/**
 * @struct ActuatorState
 * @brief Internal state of actuator dynamics filter
 *
 * The actuator model is a first-order lag filter that smooths
 * acceleration commands to simulate motor/ESC response delay:
 *   d(ax_filtered)/dt = (ax_cmd - ax_filtered) / tau
 */
struct ActuatorState {
  double ax_filtered = 0.0;  ///< Filtered acceleration X [m/s²]
  double ay_filtered = 0.0;  ///< Filtered acceleration Y [m/s²]
};

/**
 * @struct PhysicsDiagnostics
 * @brief Diagnostic output for physics debugging and analysis
 *
 * This struct captures intermediate computation results for:
 *   - Performance analysis (drag forces, actuator response)
 *   - Controller tuning (see what actuator actually produces)
 *   - Thesis plots (compare commanded vs actual acceleration)
 */
struct PhysicsDiagnostics {
  // ===== Drag Forces =====
  double ax_drag = 0.0;      ///< Drag acceleration X [m/s²]
  double ay_drag = 0.0;      ///< Drag acceleration Y [m/s²]
  double vrel_norm = 0.0;    ///< Relative velocity magnitude [m/s]

  // ===== Actuator Output =====
  double ax_filtered = 0.0;  ///< Actuator output X (post-lag filter) [m/s²]
  double ay_filtered = 0.0;  ///< Actuator output Y (post-lag filter) [m/s²]

  // ===== Wind Contribution =====
  double ax_wind_bias = 0.0;  ///< Wind bias acceleration X [m/s²]
  double ay_wind_bias = 0.0;  ///< Wind bias acceleration Y [m/s²]

  // ===== Total Acceleration =====
  double ax_total = 0.0;  ///< Total simulated acceleration X [m/s²]
  double ay_total = 0.0;  ///< Total simulated acceleration Y [m/s²]
};

/**
 * @struct FeedForwardParams
 * @brief Parameters for feed-forward control (drag/wind compensation)
 *
 * Feed-forward control predicts and cancels disturbances before they
 * affect tracking error. This improves performance without increasing
 * feedback gains (which can cause oscillations).
 */
struct FeedForwardParams {
  // ===== Enable/Disable Flags =====
  bool enable_drag_ff = false;  ///< Enable drag force cancellation
  bool enable_wind_ff = false;  ///< Enable wind bias cancellation

  // ===== Gain Factors (0.0 to 1.0+) =====
  /// Drag compensation gain [dimensionless]
  /// 0.0 = disabled, 1.0 = full cancellation, >1.0 = over-compensation
  double k_drag = 0.8;

  /// Wind bias compensation gain [dimensionless]
  /// 0.0 = disabled, 1.0 = full cancellation
  double k_wind = 1.0;

  // ===== Model Parameters (Must Match Physics!) =====
  /// Estimated mass for drag prediction [kg]
  /// Should match PhysicsParams::mass for accurate FF
  double mass_estimate = 1.5;

  /// Estimated linear drag coefficient [kg/s]
  double drag_coeff_lin_estimate = 0.12;

  /// Estimated quadratic drag coefficient [kg/m]
  double drag_coeff_quad_estimate = 0.05;

  /// Drag speed threshold estimate [m/s]
  double drag_speed_threshold_estimate = 1.0;
};

}  // namespace core
}  // namespace agent_control_pkg
