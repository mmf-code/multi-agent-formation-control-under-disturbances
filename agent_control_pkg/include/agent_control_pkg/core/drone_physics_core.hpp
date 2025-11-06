/**
 * @file drone_physics_core.hpp
 * @brief Header-only 2D drone physics computation core
 *
 * This library implements the complete 2D quadrotor dynamics model used
 * in both standalone simulations and ROS2/Gazebo integration. All functions
 * are inline/template to avoid link-time conflicts.
 *
 * Physics Model Overview:
 *   1. Actuator Dynamics: First-order lag with asymmetric time constants
 *   2. Drag Forces: Smooth blend from linear to quadratic with speed
 *   3. Wind Disturbances: Velocity-based (affects drag) + constant bias
 *   4. Total Acceleration: actuator_output + drag + wind_bias
 *
 * Design Philosophy:
 *   - Header-only: All functions inline
 *   - Zero dependencies: Pure C++ math only
 *   - Deterministic: Same inputs → same outputs (no hidden state)
 *   - Validated: Matches drone_dynamics_2d.cpp line-by-line
 *
 * @author Multi-Agent Formation Control Team
 * @date 2025
 */

#pragma once

#include "physics_types.hpp"
#include <cmath>
#include <algorithm>

namespace agent_control_pkg {
namespace core {

/**
 * @class DronePhysicsCore
 * @brief Static functions for 2D drone physics computation
 *
 * All functions are static and stateless for easy testing and reuse.
 */
class DronePhysicsCore {
public:
  /**
   * @brief Update actuator state with first-order lag filter (asymmetric)
   * @param ax_cmd Commanded acceleration X [m/s²]
   * @param ay_cmd Commanded acceleration Y [m/s²]
   * @param actuator_state Actuator filter state (modified in-place)
   * @param params Physics parameters (time constants)
   * @param dt Time step [s]
   *
   * Implements asymmetric first-order lag:
   *   - tau_up for rising edge (command > current)
   *   - tau_down for falling edge (command < current)
   *
   * This models motor/ESC response delay. Typically tau_down > tau_up
   * because motors spin down slower than they spin up.
   *
   * Discretization: Semi-implicit Euler
   *   ax_filtered_new = ax_filtered_old + (ax_cmd - ax_filtered_old) * (dt/tau)
   */
  static inline void computeActuatorFiltering(
    double ax_cmd,
    double ay_cmd,
    ActuatorState& actuator_state,
    const PhysicsParams& params,
    double dt)
  {
    // Clamp commands to max acceleration limits
    ax_cmd = std::clamp(ax_cmd, -params.max_accel, params.max_accel);
    ay_cmd = std::clamp(ay_cmd, -params.max_accel, params.max_accel);

    // X-axis: Asymmetric time constant selection
    const double tau_x = (ax_cmd > actuator_state.ax_filtered) ?
      (params.actuator_tau_up > 0.0 ? params.actuator_tau_up : params.actuator_tau) :
      (params.actuator_tau_down > 0.0 ? params.actuator_tau_down : params.actuator_tau);

    // Y-axis: Asymmetric time constant selection
    const double tau_y = (ay_cmd > actuator_state.ay_filtered) ?
      (params.actuator_tau_up > 0.0 ? params.actuator_tau_up : params.actuator_tau) :
      (params.actuator_tau_down > 0.0 ? params.actuator_tau_down : params.actuator_tau);

    // First-order filter update (discrete-time approximation)
    if (tau_x > 1e-6) {
      const double alpha_x = dt / tau_x;
      actuator_state.ax_filtered += alpha_x * (ax_cmd - actuator_state.ax_filtered);
    } else {
      actuator_state.ax_filtered = ax_cmd;  // No filtering if tau too small
    }

    if (tau_y > 1e-6) {
      const double alpha_y = dt / tau_y;
      actuator_state.ay_filtered += alpha_y * (ay_cmd - actuator_state.ay_filtered);
    } else {
      actuator_state.ay_filtered = ay_cmd;  // No filtering if tau too small
    }
  }

  /**
   * @brief Compute drag acceleration with linear→quadratic blend
   * @param vx Drone velocity X [m/s]
   * @param vy Drone velocity Y [m/s]
   * @param wind Ambient wind environment
   * @param params Physics parameters (drag coefficients)
   * @param ax_drag_out Output: drag acceleration X [m/s²]
   * @param ay_drag_out Output: drag acceleration Y [m/s²]
   * @param vrel_norm_out Output: relative velocity magnitude [m/s]
   *
   * Drag Model:
   *   1. Compute relative velocity: v_rel = v_drone - v_wind
   *   2. At low speeds (< threshold): F_drag = -cd_lin * v_rel
   *   3. At high speeds (> threshold): F_drag = -cd_quad * |v_rel| * v_rel
   *   4. Smooth blend in transition region using weight function
   *
   * Blend Function:
   *   w = clamp((|v_rel| - v_thr) / v_thr, 0, 1)
   *   F_drag = (1-w)*F_linear + w*F_quadratic
   *
   * This matches real aerodynamics: laminar flow at low speeds,
   * turbulent flow at high speeds.
   */
  static inline void computeDragAcceleration(
    double vx,
    double vy,
    const WindEnvironment& wind,
    const PhysicsParams& params,
    double& ax_drag_out,
    double& ay_drag_out,
    double& vrel_norm_out)
  {
    // Relative velocity (drone velocity minus wind)
    const double vrel_x = vx - wind.vx;
    const double vrel_y = vy - wind.vy;
    const double vrel_norm = std::sqrt(vrel_x * vrel_x + vrel_y * vrel_y);
    vrel_norm_out = vrel_norm;

    // Inverse mass for acceleration computation
    const double inv_m = 1.0 / params.mass;

    // Speed threshold for blend transition
    const double v_thr = params.drag_speed_threshold;

    // Lambda function for per-axis drag calculation
    auto drag_axis = [&](double v_component) -> double {
      // Linear drag term: F = -cd_lin * v
      const double a_lin = -(params.drag_coeff_lin * inv_m) * v_component;

      // Quadratic drag term: F = -cd_quad * |v| * v
      const double a_quad = -(params.drag_coeff_quad * inv_m) * vrel_norm * v_component;

      // Blend weight (0 at low speed, 1 at high speed)
      const double w = std::clamp(
        (vrel_norm - v_thr) / std::max(v_thr, 1e-6),
        0.0, 1.0
      );

      // Smooth blend: linear at low speed, quadratic at high speed
      return (1.0 - w) * a_lin + w * a_quad;
    };

    // Compute drag for each axis independently
    ax_drag_out = drag_axis(vrel_x);
    ay_drag_out = drag_axis(vrel_y);
  }

  /**
   * @brief Complete physics step: actuator + drag + wind bias
   * @param ax_cmd Commanded acceleration X [m/s²]
   * @param ay_cmd Commanded acceleration Y [m/s²]
   * @param vx Current velocity X [m/s]
   * @param vy Current velocity Y [m/s]
   * @param actuator_state Actuator filter state (modified in-place)
   * @param wind Wind environment
   * @param params Physics parameters
   * @param dt Time step [s]
   * @param ax_total_out Output: total acceleration X [m/s²]
   * @param ay_total_out Output: total acceleration Y [m/s²]
   * @param diag_out Optional diagnostics output
   *
   * Full Physics Pipeline:
   *   1. Actuator filtering: ax_cmd → ax_filtered (first-order lag)
   *   2. Drag computation: f(velocity, wind) → ax_drag
   *   3. Wind bias: constant acceleration disturbance
   *   4. Total: ax_total = ax_filtered + ax_drag + ax_wind_bias
   *
   * This function is the single source of truth for 2D drone physics.
   */
  static inline void computePhysicsStep(
    double ax_cmd,
    double ay_cmd,
    double vx,
    double vy,
    ActuatorState& actuator_state,
    const WindEnvironment& wind,
    const PhysicsParams& params,
    double dt,
    double& ax_total_out,
    double& ay_total_out,
    PhysicsDiagnostics* diag_out = nullptr)
  {
    // Step 1: Actuator dynamics (first-order lag with asymmetric tau)
    computeActuatorFiltering(ax_cmd, ay_cmd, actuator_state, params, dt);

    // Step 2: Drag forces (linear→quadratic blend)
    double ax_drag, ay_drag, vrel_norm;
    computeDragAcceleration(vx, vy, wind, params, ax_drag, ay_drag, vrel_norm);

    // Step 3: Total acceleration = actuator + drag + wind_bias
    ax_total_out = actuator_state.ax_filtered + ax_drag + wind.ax_bias;
    ay_total_out = actuator_state.ay_filtered + ay_drag + wind.ay_bias;

    // Step 4: Fill diagnostics if requested
    if (diag_out) {
      diag_out->ax_drag = ax_drag;
      diag_out->ay_drag = ay_drag;
      diag_out->vrel_norm = vrel_norm;
      diag_out->ax_filtered = actuator_state.ax_filtered;
      diag_out->ay_filtered = actuator_state.ay_filtered;
      diag_out->ax_wind_bias = wind.ax_bias;
      diag_out->ay_wind_bias = wind.ay_bias;
      diag_out->ax_total = ax_total_out;
      diag_out->ay_total = ay_total_out;
    }
  }

  // ========== FEED-FORWARD PREDICTION FUNCTIONS ==========

  /**
   * @brief Predict drag acceleration for feed-forward control
   * @param vx Predicted/current velocity X [m/s]
   * @param vy Predicted/current velocity Y [m/s]
   * @param wind Wind environment
   * @param ff_params Feed-forward parameters (must match physics!)
   * @param ax_ff_out Output: predicted drag acceleration X [m/s²]
   * @param ay_ff_out Output: predicted drag acceleration Y [m/s²]
   *
   * Feed-Forward Drag Cancellation:
   *   The controller predicts drag forces and adds compensation:
   *     ax_cmd_final = ax_pid + (-k_drag * ax_drag_predicted)
   *
   * Critical Requirement:
   *   ff_params (mass, drag coeffs) MUST match PhysicsParams exactly!
   *   Mismatch → imperfect cancellation → residual error
   */
  static inline void predictDragForFeedForward(
    double vx,
    double vy,
    const WindEnvironment& wind,
    const FeedForwardParams& ff_params,
    double& ax_ff_out,
    double& ay_ff_out)
  {
    // Relative velocity
    const double vrel_x = vx - wind.vx;
    const double vrel_y = vy - wind.vy;
    const double vrel_norm = std::sqrt(vrel_x * vrel_x + vrel_y * vrel_y);

    const double inv_m = 1.0 / ff_params.mass_estimate;
    const double v_thr = ff_params.drag_speed_threshold_estimate;

    // Per-axis drag prediction (same logic as computeDragAcceleration)
    auto drag_axis_pred = [&](double v_component) -> double {
      const double a_lin = -(ff_params.drag_coeff_lin_estimate * inv_m) * v_component;
      const double a_quad = -(ff_params.drag_coeff_quad_estimate * inv_m) * vrel_norm * v_component;
      const double w = std::clamp((vrel_norm - v_thr) / std::max(v_thr, 1e-6), 0.0, 1.0);
      return (1.0 - w) * a_lin + w * a_quad;
    };

    ax_ff_out = drag_axis_pred(vrel_x);
    ay_ff_out = drag_axis_pred(vrel_y);
  }

  /**
   * @brief Compute complete feed-forward compensation
   * @param vx Current velocity X [m/s]
   * @param vy Current velocity Y [m/s]
   * @param wind Wind environment
   * @param ff_params Feed-forward configuration
   * @param ax_ff_out Output: feed-forward compensation X [m/s²]
   * @param ay_ff_out Output: feed-forward compensation Y [m/s²]
   *
   * Feed-Forward Components:
   *   1. Drag cancellation: -k_drag * predicted_drag
   *   2. Wind bias cancellation: -k_wind * wind_bias
   *
   * Usage in controller:
   *   ax_cmd_total = ax_pid + ax_ff
   */
  static inline void computeFeedForwardCompensation(
    double vx,
    double vy,
    const WindEnvironment& wind,
    const FeedForwardParams& ff_params,
    double& ax_ff_out,
    double& ay_ff_out)
  {
    ax_ff_out = 0.0;
    ay_ff_out = 0.0;

    // Component 1: Drag cancellation
    if (ff_params.enable_drag_ff) {
      double ax_drag_pred, ay_drag_pred;
      predictDragForFeedForward(vx, vy, wind, ff_params, ax_drag_pred, ay_drag_pred);
      ax_ff_out += -ff_params.k_drag * ax_drag_pred;
      ay_ff_out += -ff_params.k_drag * ay_drag_pred;
    }

    // Component 2: Wind bias cancellation
    if (ff_params.enable_wind_ff) {
      ax_ff_out += -ff_params.k_wind * wind.ax_bias;
      ay_ff_out += -ff_params.k_wind * wind.ay_bias;
    }
  }

};  // class DronePhysicsCore

}  // namespace core
}  // namespace agent_control_pkg
