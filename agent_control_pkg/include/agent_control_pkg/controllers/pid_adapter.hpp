#ifndef AGENT_CONTROL_PKG__CONTROLLERS__PID_ADAPTER_HPP_
#define AGENT_CONTROL_PKG__CONTROLLERS__PID_ADAPTER_HPP_

#include "agent_control_pkg/controllers/controller_base.hpp"
#include "agent_control_pkg/pid_controller.hpp"

namespace agent_control_pkg::controllers {

class PIDAdapter : public IController1D {
public:
  PIDAdapter(double kp, double ki, double kd, double umin, double umax)
  : pid_(kp, ki, kd, umin, umax) {}

  void setGains(double kp, double ki, double kd) { pid_.setTunings(kp, ki, kd); }
  void setLimits(double umin, double umax) { pid_.setOutputLimits(umin, umax); }
  void enableDerivativeFilter(bool enable, double alpha) { pid_.enableDerivativeFilter(enable, alpha); }

  // Anti-windup configuration
  void setAntiWindupMode(PIDController::AntiWindupMode mode) { pid_.setAntiWindupMode(mode); }
  void setTrackingTimeConstant(double Tt) { pid_.setTrackingTimeConstant(Tt); }

  double compute(double y, double yref, double dt) override {
    pid_.setSetpoint(yref);
    return pid_.calculate(y, dt);
  }

  void reset() override { pid_.reset(); }

  const agent_control_pkg::PIDController& pid() const { return pid_; }

private:
  agent_control_pkg::PIDController pid_;
};

} // namespace agent_control_pkg::controllers

#endif // AGENT_CONTROL_PKG__CONTROLLERS__PID_ADAPTER_HPP_

