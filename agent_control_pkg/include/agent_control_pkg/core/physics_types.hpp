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
 * of the quadrotor. Default values are for Crazyflie 2.1 nano quadrotor.
 *
 * IMPORTANT: When using with Gazebo, mass is read from SDF model inertial.
 * Other parameters can be overridden via SDF plugin elements.
 */
struct PhysicsParams {
  // ===== Inertial Properties =====
  /// Drone mass [kg] - Crazyflie 2.1 default: 27g
  double mass = 0.027;

  // ===== Actuator Dynamics (First-Order Lag) =====
  // Values derived from sim_cf2 Crazyflie motor model (CrazyflieTHI/sim_cf2)
  // Motor time constants from crazyflie.xacro: time_constant_up=0.0125, time_constant_down=0.025

  /// Symmetric time constant [s] (used if asymmetric not set)
  /// Average of up/down for simplified models
  double actuator_tau = 0.02;

  /// Rising edge time constant [s] (motor spin-up)
  /// sim_cf2 value: 12.5ms - motors spin up fast
  double actuator_tau_up = 0.0125;

  /// Falling edge time constant [s] (motor spin-down)
  /// sim_cf2 value: 25ms - motors spin down slower due to inertia
  double actuator_tau_down = 0.025;

  /// Maximum acceleration command [m/s²] (saturation limit)
  /// Derived from sim_cf2: 4 motors × 0.119N = 0.476N total thrust
  /// max_accel = (T_total - m*g) / m = (0.476 - 0.265) / 0.027 ≈ 7.8 m/s² (sustained)
  /// Peak (all thrust for accel): 0.476 / 0.027 = 17.6 m/s²
  /// Using 17.0 for realistic ceiling with some margin
  double max_accel = 17.0;

  // ===== Drag Model (Linear → Quadratic Blend) =====
  /// Linear drag coefficient [N/(m/s)] or equivalently [kg/s]
  /// Force = -cd_lin * v_rel
  /// Reduced slightly for Crazyflie's small profile (was 0.005)
  double drag_coeff_lin = 0.003;

  /// Quadratic drag coefficient [N/(m/s)²] or [kg/m]
  /// Force = -cd_quad * |v_rel| * v_rel
  /// Crazyflie: Cd*A ≈ 1.2 * 0.001 = 0.0012, so 0.5*rho*Cd*A ≈ 0.0008
  double drag_coeff_quad = 0.0008;

  /// Speed threshold [m/s] for drag model transition
  /// Below threshold: linear drag dominates
  /// Above threshold: quadratic drag dominates
  /// Smooth blend in between using sigmoid-like function
  double drag_speed_threshold = 0.5;

  // ===== Diagnostic Flags =====
  /// Enable detailed physics logging (for debugging)
  bool enable_diagnostics = false;

  // ===== Factory Methods for Common Drone Types =====
  /// Crazyflie 2.1 parameters derived from sim_cf2 (CrazyflieTHI/sim_cf2)
  static PhysicsParams crazyflie() {
    PhysicsParams p;
    p.mass = 0.027;              // 27g
    p.actuator_tau = 0.02;       // Average of up/down
    p.actuator_tau_up = 0.0125;  // 12.5ms (sim_cf2)
    p.actuator_tau_down = 0.025; // 25ms (sim_cf2)
    p.max_accel = 17.0;          // ~17.6 m/s² from thrust calculation
    p.drag_coeff_lin = 0.003;    // Reduced for nano quadrotor
    p.drag_coeff_quad = 0.0008;  // Reduced for small frontal area
    p.drag_speed_threshold = 0.5;
    return p;
  }

  static PhysicsParams generic_1_5kg() {
    PhysicsParams p;
    p.mass = 1.5;
    p.actuator_tau = 0.1;
    p.actuator_tau_up = 0.1;
    p.actuator_tau_down = 0.15;
    p.max_accel = 10.0;
    p.drag_coeff_lin = 0.12;
    p.drag_coeff_quad = 0.05;
    p.drag_speed_threshold = 1.0;
    return p;
  }
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
 *
 * Default values are for Crazyflie 2.1 nano quadrotor.
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
  /// Default: Crazyflie 2.1 mass (27g)
  double mass_estimate = 0.027;

  /// Estimated linear drag coefficient [kg/s]
  /// Default: Crazyflie value (matched to physics_types.hpp)
  double drag_coeff_lin_estimate = 0.003;

  /// Estimated quadratic drag coefficient [kg/m]
  /// Default: Crazyflie value (matched to physics_types.hpp)
  double drag_coeff_quad_estimate = 0.0008;

  /// Drag speed threshold estimate [m/s]
  double drag_speed_threshold_estimate = 0.5;

  // ===== Factory Methods =====
  /// Crazyflie 2.1 FF params (must match PhysicsParams::crazyflie())
  static FeedForwardParams crazyflie() {
    FeedForwardParams p;
    p.mass_estimate = 0.027;
    p.drag_coeff_lin_estimate = 0.003;   // Matched to PhysicsParams
    p.drag_coeff_quad_estimate = 0.0008; // Matched to PhysicsParams
    p.drag_speed_threshold_estimate = 0.5;
    return p;
  }

  static FeedForwardParams generic_1_5kg() {
    FeedForwardParams p;
    p.mass_estimate = 1.5;
    p.drag_coeff_lin_estimate = 0.12;
    p.drag_coeff_quad_estimate = 0.05;
    p.drag_speed_threshold_estimate = 1.0;
    return p;
  }
};

}  // namespace core
}  // namespace agent_control_pkg
