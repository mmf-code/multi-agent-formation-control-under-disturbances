#ifndef AGENT_CONTROL_PKG__CONTROLLERS__FUZZY_IT2_ADAPTER_HPP_
#define AGENT_CONTROL_PKG__CONTROLLERS__FUZZY_IT2_ADAPTER_HPP_

/**
 * @file fuzzy_it2_adapter.hpp
 * @brief Adapter for IT2 Fuzzy Logic System as IController1D
 *
 * Bridges the IT2FuzzyLogicSystem to the controller interface,
 * handling error/dError calculation and output saturation.
 */

#include "agent_control_pkg/controllers/controller_base.hpp"
#include "agent_control_pkg/it2_fuzzy_logic_system.hpp"
#include <memory>
#include <string>
#include <map>
#include <array>
#include <vector>
#include "agent_control_pkg/config_reader.hpp"

namespace agent_control_pkg::controllers {

/**
 * @class FuzzyIT2Adapter
 * @brief Controller adapter for Interval Type-2 Fuzzy Logic System
 */
class FuzzyIT2Adapter : public IController1D {
public:
  struct Options {
    double umin{-12.0};       ///< Output lower limit
    double umax{12.0};        ///< Output upper limit
    double wind_scalar{0.0};  ///< Wind disturbance scalar for fuzzy input
  };

  explicit FuzzyIT2Adapter(const Options &opt);

  /**
   * @brief Configure fuzzy system from parsed parameters
   * @param sets Map of variable name -> set name -> FOU parameters
   * @param rules Vector of rules: [error_label, dError_label, wind_label, output_label]
   * @param include_wind Whether to include wind as fuzzy input
   */
  void configureFromParams(
      const std::map<std::string,
                     std::map<std::string, IT2FuzzyLogicSystem::IT2TriangularFS_FOU>> &sets,
      const std::vector<std::array<std::string, 4>> &rules,
      bool include_wind);

  /**
   * @brief Configure from FuzzyParams structure
   * @param fp Parsed fuzzy parameters from YAML
   * @param include_wind Whether to include wind input
   */
  void configureFromFuzzyParams(const FuzzyParams &fp, bool include_wind);

  /**
   * @brief Compute control output
   * @param y Current measurement
   * @param yref Reference setpoint
   * @param dt Time step (s)
   * @return Control command (saturated)
   */
  double compute(double y, double yref, double dt) override;

  void reset() override;

  void setWindScalar(double w) { opt_.wind_scalar = w; }
  void setLimits(double umin, double umax) { opt_.umin = umin; opt_.umax = umax; }

  /// Get reference to internal IT2-FLS for inspection
  const IT2FuzzyLogicSystem& fls() const { return fls_; }

private:
  Options opt_;
  IT2FuzzyLogicSystem fls_;
  bool include_wind_{false};
  bool initialized_{false};
  double prev_error_{0.0};
  bool first_{true};
};

// Backward compatibility alias
using FuzzyGT2Adapter = FuzzyIT2Adapter;

} // namespace agent_control_pkg::controllers

#endif // AGENT_CONTROL_PKG__CONTROLLERS__FUZZY_IT2_ADAPTER_HPP_
