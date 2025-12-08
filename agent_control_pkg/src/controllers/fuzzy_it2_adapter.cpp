/**
 * @file fuzzy_it2_adapter.cpp
 * @brief Implementation of FuzzyIT2Adapter
 */

#include "agent_control_pkg/controllers/fuzzy_it2_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace agent_control_pkg::controllers {

FuzzyIT2Adapter::FuzzyIT2Adapter(const Options &opt)
  : opt_(opt)
{
  reset();
}

void FuzzyIT2Adapter::configureFromParams(
    const std::map<std::string,
                   std::map<std::string, IT2FuzzyLogicSystem::IT2TriangularFS_FOU>> &sets,
    const std::vector<std::array<std::string, 4>> &rules,
    bool include_wind)
{
  // Reinitialize internal FLS state
  fls_ = IT2FuzzyLogicSystem{};
  include_wind_ = include_wind;

  // Detect output variable name from provided sets
  // Prefer "correction" (as used by default config), fallback to "output"
  std::string output_var = "";
  if (sets.count("correction")) {
    output_var = "correction";
  } else if (sets.count("output")) {
    output_var = "output";
  } else {
    // Fallback: pick the first var not among known inputs
    for (const auto &kv : sets) {
      if (kv.first != "error" && kv.first != "dError" && kv.first != "wind") {
        output_var = kv.first;
        break;
      }
    }
    if (output_var.empty()) {
      output_var = "output";
    }
  }

  // Declare variables in the FLS
  fls_.addInputVariable("error");
  fls_.addInputVariable("dError");
  if (include_wind_) {
    fls_.addInputVariable("wind");
  }
  fls_.addOutputVariable(output_var);

  // Load fuzzy sets for all variables
  for (const auto &var_entry : sets) {
    const std::string &var_name = var_entry.first;
    for (const auto &set_entry : var_entry.second) {
      const std::string &set_name = set_entry.first;
      const auto &fou = set_entry.second;
      fls_.addFuzzySetToVariable(var_name, set_name, fou);
    }
  }

  // Add rules: [error_label, dError_label, wind_label, output_label]
  for (const auto &r : rules) {
    IT2FuzzyLogicSystem::FuzzyRule rule;
    rule.antecedents.clear();
    rule.antecedents["error"] = r[0];
    rule.antecedents["dError"] = r[1];
    if (include_wind_) {
      rule.antecedents["wind"] = r[2];
    }
    rule.consequent = {output_var, r[3]};
    fls_.addRule(rule);
  }

  initialized_ = true;
}

void FuzzyIT2Adapter::configureFromFuzzyParams(const FuzzyParams &fp,
                                               bool include_wind)
{
  // Convert FuzzyParams::FuzzySetFOU -> IT2FuzzyLogicSystem::IT2TriangularFS_FOU
  std::map<std::string,
           std::map<std::string, IT2FuzzyLogicSystem::IT2TriangularFS_FOU>> converted_sets;

  for (const auto &var_pair : fp.sets) {
    const std::string &var_name = var_pair.first;
    for (const auto &set_pair : var_pair.second) {
      const std::string &set_name = set_pair.first;
      const auto &s = set_pair.second;  // FuzzySetFOU {l1,l2,l3,u1,u2,u3}
      IT2FuzzyLogicSystem::IT2TriangularFS_FOU fou{
          s.l1, s.l2, s.l3,
          s.u1, s.u2, s.u3
      };
      converted_sets[var_name][set_name] = fou;
    }
  }

  configureFromParams(converted_sets, fp.rules, include_wind);
}

double FuzzyIT2Adapter::compute(double y, double yref, double dt)
{
  if (!initialized_) {
    return 0.0;
  }

  // Compute error and derivative of error
  const double error = yref - y;
  double derr = 0.0;
  const double safe_dt = (dt > 1e-9 && std::isfinite(dt)) ? dt : 1e-3;

  if (first_) {
    derr = 0.0;
    first_ = false;
  } else {
    derr = (error - prev_error_) / safe_dt;
  }
  prev_error_ = error;

  // Wind scalar is a simple magnitude proxy supplied via options
  const double wind_val = include_wind_ ? opt_.wind_scalar : 0.0;

  // Evaluate IT2-FLS
  double u = fls_.calculateOutput(error, derr, wind_val);

  // Handle NaN/Inf
  if (!std::isfinite(u)) {
    u = 0.0;
  }

  // Clamp to actuator limits
  u = std::clamp(u, opt_.umin, opt_.umax);

  return u;
}

void FuzzyIT2Adapter::reset()
{
  first_ = true;
  prev_error_ = 0.0;
}

} // namespace agent_control_pkg::controllers
