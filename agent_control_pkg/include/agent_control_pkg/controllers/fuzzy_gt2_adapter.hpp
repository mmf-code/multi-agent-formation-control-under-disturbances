#ifndef AGENT_CONTROL_PKG__CONTROLLERS__FUZZY_GT2_ADAPTER_HPP_
#define AGENT_CONTROL_PKG__CONTROLLERS__FUZZY_GT2_ADAPTER_HPP_

#include "agent_control_pkg/controllers/controller_base.hpp"
#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include <memory>
#include <string>
#include <map>
#include <array>
#include <vector>
#include "agent_control_pkg/config_reader.hpp" // for FuzzyParams

namespace agent_control_pkg::controllers {

class FuzzyGT2Adapter : public IController1D {
public:
  struct Options {
    double umin{-12.0};
    double umax{12.0};
    double wind_scalar{0.0}; // e.g., ||v_wind||
  };

  FuzzyGT2Adapter(const Options &opt);

  // Build fuzzy sets and rules from parsed params. Variables are expected as
  // "error", "dError", optional "wind", and output "output".
  void configureFromParams(const std::map<std::string, std::map<std::string, agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU>> &sets,
                           const std::vector<std::array<std::string,4>> &rules,
                           bool include_wind);

  // Convenience: configure directly from existing FuzzyParams (keeps your loader intact)
  void configureFromFuzzyParams(const agent_control_pkg::FuzzyParams &fp,
                                bool include_wind);

  double compute(double y, double yref, double dt) override;
  void reset() override;

  void setWindScalar(double w) { opt_.wind_scalar = w; }
  void setLimits(double umin, double umax) { opt_.umin = umin; opt_.umax = umax; }

private:
  Options opt_;
  agent_control_pkg::GT2FuzzyLogicSystem fls_;
  bool include_wind_{false};
  bool initialized_{false};
  double prev_error_{0.0};
  bool first_{true};
};

} // namespace agent_control_pkg::controllers

#endif // AGENT_CONTROL_PKG__CONTROLLERS__FUZZY_GT2_ADAPTER_HPP_
