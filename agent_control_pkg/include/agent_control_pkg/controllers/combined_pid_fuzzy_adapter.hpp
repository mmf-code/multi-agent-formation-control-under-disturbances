#ifndef AGENT_CONTROL_PKG__CONTROLLERS__COMBINED_PID_FUZZY_ADAPTER_HPP_
#define AGENT_CONTROL_PKG__CONTROLLERS__COMBINED_PID_FUZZY_ADAPTER_HPP_

#include "agent_control_pkg/controllers/controller_base.hpp"
#include "agent_control_pkg/controllers/pid_adapter.hpp"
#include "agent_control_pkg/controllers/fuzzy_gt2_adapter.hpp"
#include <memory>

namespace agent_control_pkg::controllers {

class CombinedPidFuzzyAdapter : public IController1D {
public:
  CombinedPidFuzzyAdapter(std::unique_ptr<PIDAdapter> pid,
                          std::unique_ptr<FuzzyGT2Adapter> fuzzy,
                          double k_pid,
                          double k_fuzzy,
                          double umin,
                          double umax)
    : pid_(std::move(pid)), fuzzy_(std::move(fuzzy)),
      k_pid_(k_pid), k_fuzzy_(k_fuzzy), umin_(umin), umax_(umax) {}

  double compute(double y, double yref, double dt) override {
    const double u_pid = pid_ ? pid_->compute(y, yref, dt) : 0.0;
    const double u_f   = fuzzy_ ? fuzzy_->compute(y, yref, dt) : 0.0;
    double u = k_pid_ * u_pid + k_fuzzy_ * u_f;
    if (u < umin_) u = umin_;
    if (u > umax_) u = umax_;
    return u;
  }

  void reset() override {
    if (pid_) pid_->reset();
    if (fuzzy_) fuzzy_->reset();
  }

  void setMix(double kpid, double kfuzzy) { k_pid_ = kpid; k_fuzzy_ = kfuzzy; }

private:
  std::unique_ptr<PIDAdapter> pid_;
  std::unique_ptr<FuzzyGT2Adapter> fuzzy_;
  double k_pid_{1.0};
  double k_fuzzy_{1.0};
  double umin_{};
  double umax_{};
};

} // namespace agent_control_pkg::controllers

#endif // AGENT_CONTROL_PKG__CONTROLLERS__COMBINED_PID_FUZZY_ADAPTER_HPP_

