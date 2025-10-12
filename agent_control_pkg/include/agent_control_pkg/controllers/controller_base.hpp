#ifndef AGENT_CONTROL_PKG__CONTROLLERS__CONTROLLER_BASE_HPP_
#define AGENT_CONTROL_PKG__CONTROLLERS__CONTROLLER_BASE_HPP_

namespace agent_control_pkg::controllers {

class IController1D {
public:
  virtual ~IController1D() = default;
  // Compute control command for measured y toward reference yref with step dt (seconds)
  virtual double compute(double y, double yref, double dt) = 0;
  virtual void reset() = 0;
};

} // namespace agent_control_pkg::controllers

#endif // AGENT_CONTROL_PKG__CONTROLLERS__CONTROLLER_BASE_HPP_

