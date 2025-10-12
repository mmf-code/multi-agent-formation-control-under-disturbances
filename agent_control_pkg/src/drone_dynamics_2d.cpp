#include "agent_control_pkg/drone_dynamics_2d.hpp"
#include <algorithm>
#include <cmath>

namespace agent_control_pkg {

DroneDynamics2D::DroneDynamics2D() = default;

void DroneDynamics2D::setParams(const Params &p) { params_ = p; }
const DroneDynamics2D::Params &DroneDynamics2D::getParams() const { return params_; }

void DroneDynamics2D::setState(const State &s) { state_ = s; }
const DroneDynamics2D::State &DroneDynamics2D::getState() const { return state_; }

void DroneDynamics2D::setWindAccel(double ax_wind, double ay_wind) {
  ax_w_ = ax_wind;
  ay_w_ = ay_wind;
}

void DroneDynamics2D::setWindVelocity(double vx_wind, double vy_wind) {
  vx_w_ = vx_wind;
  vy_w_ = vy_wind;
}

void DroneDynamics2D::step(double ax_cmd, double ay_cmd, double dt) {
  if (dt <= 0.0) return;

  // First-order actuator filtering (support asymmetric time constants)
  const double tau_x = (ax_cmd > ax_cmd_f_) ?
      (params_.actuator_tau_up > 0.0 ? params_.actuator_tau_up : params_.actuator_tau)
      : (params_.actuator_tau_down > 0.0 ? params_.actuator_tau_down : params_.actuator_tau);
  const double tau_y = (ay_cmd > ay_cmd_f_) ?
      (params_.actuator_tau_up > 0.0 ? params_.actuator_tau_up : params_.actuator_tau)
      : (params_.actuator_tau_down > 0.0 ? params_.actuator_tau_down : params_.actuator_tau);

  const double alpha_x = std::clamp(dt / std::max(tau_x, 1e-6), 0.0, 1.0);
  const double alpha_y = std::clamp(dt / std::max(tau_y, 1e-6), 0.0, 1.0);
  ax_cmd_f_ += alpha_x * (ax_cmd - ax_cmd_f_);
  ay_cmd_f_ += alpha_y * (ay_cmd - ay_cmd_f_);

  // Per-axis acceleration saturation
  ax_cmd_f_ = std::clamp(ax_cmd_f_, -params_.max_accel, params_.max_accel);
  ay_cmd_f_ = std::clamp(ay_cmd_f_, -params_.max_accel, params_.max_accel);

    // Relative airspeed (ambient wind velocity)
  const double vx_rel = state_.vx - vx_w_;
  const double vy_rel = state_.vy - vy_w_;
  const double vrel_norm = std::hypot(vx_rel, vy_rel);
  last_vrel_norm_ = vrel_norm;

  // Drag: linear at low speeds, blends toward quadratic at higher speeds using |v| for quadratic term
  const double inv_m = (params_.mass > 1e-9) ? (1.0 / params_.mass) : 0.0;
  const double v_thr = std::max(0.0, params_.drag_speed_threshold);

  auto drag_axis = [&](double v_component) {
    const double a_lin = -(params_.drag_coeff_lin * inv_m) * v_component;
    if (params_.drag_coeff_quad <= 0.0) {
      return a_lin; // pure linear if no quadratic coeff
    }
    // Quadratic uses |v|*v_component with |v| = ||v_rel|| to capture cross-coupling
    const double a_quad = -(params_.drag_coeff_quad * inv_m) * vrel_norm * v_component;
    if (v_thr <= 1e-9 || vrel_norm <= v_thr) {
      return a_lin;
    }
    const double w = std::clamp((vrel_norm - v_thr) / std::max(v_thr, 1e-6), 0.0, 1.0);
    return (1.0 - w) * a_lin + w * a_quad; // smooth blend on speed norm
  };

  const double ax_drag = drag_axis(vx_rel);
  const double ay_drag = drag_axis(vy_rel);
  last_ax_drag_ = ax_drag;
  last_ay_drag_ = ay_drag;const double ax = ax_cmd_f_ + ax_drag + ax_w_;
  const double ay = ay_cmd_f_ + ay_drag + ay_w_;

  // Integrate (semi-implicit Euler)
  state_.vx += ax * dt;
  state_.vy += ay * dt;
  state_.x += state_.vx * dt;
  state_.y += state_.vy * dt;
}

} // namespace agent_control_pkg

