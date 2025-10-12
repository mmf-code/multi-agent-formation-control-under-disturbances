#ifndef AGENT_CONTROL_PKG__DRONE_DYNAMICS_2D_HPP_
#define AGENT_CONTROL_PKG__DRONE_DYNAMICS_2D_HPP_

namespace agent_control_pkg {

class DroneDynamics2D {
public:
  struct Params {
    double mass{1.5};                // kg
    double drag_coeff_lin{0.1};      // linear drag coefficient (kg/s)
    double drag_coeff_quad{0.0};     // quadratic drag coefficient (kg/m)
    double drag_speed_threshold{2.0}; // [m/s] threshold between linear/quadratic regimes
    double max_accel{15.0};          // m/s^2 per-axis limit
    double actuator_tau{0.1};        // s (legacy symmetric time constant)
    double actuator_tau_up{0.1};     // s (asymmetric: ramp-up)
    double actuator_tau_down{0.1};   // s (asymmetric: ramp-down)
  };

  struct State {
    double x{0.0}, y{0.0};
    double vx{0.0}, vy{0.0};
  };

  DroneDynamics2D();

  void setParams(const Params &p);
  const Params &getParams() const;

  void setState(const State &s);
  const State &getState() const;

  // Constant wind acceleration (m/s^2); set to zero to disable
  void setWindAccel(double ax_wind, double ay_wind);

  // Ambient wind velocity (m/s); drag is computed using relative airspeed v_rel = v - v_wind
  void setWindVelocity(double vx_wind, double vy_wind);

  // Step simulation by dt with commanded acceleration (m/s^2)
  // Applies actuator first-order filter, per-axis saturation, linear drag, wind
  void step(double ax_cmd, double ay_cmd, double dt);

  // Diagnostics (values from the last step)
  inline void getFilteredCommand(double &ax_cmd_f, double &ay_cmd_f) const {
    ax_cmd_f = ax_cmd_f_;
    ay_cmd_f = ay_cmd_f_;
  }
  inline void getDragAccel(double &ax_drag, double &ay_drag) const {
    ax_drag = last_ax_drag_;
    ay_drag = last_ay_drag_;
  }
  inline double getRelativeAirSpeedNorm() const { return last_vrel_norm_; }

private:
  Params params_{};
  State state_{};

  // Actuator filtered command (m/s^2)
  double ax_cmd_f_{0.0}, ay_cmd_f_{0.0};

  // Wind acceleration (m/s^2)
  double ax_w_{0.0}, ay_w_{0.0};

  // Wind ambient velocity (m/s)
  double vx_w_{0.0}, vy_w_{0.0};

  // Diagnostics
  double last_ax_drag_{0.0}, last_ay_drag_{0.0};
  double last_vrel_norm_{0.0};
};

} // namespace agent_control_pkg

#endif // AGENT_CONTROL_PKG__DRONE_DYNAMICS_2D_HPP_
