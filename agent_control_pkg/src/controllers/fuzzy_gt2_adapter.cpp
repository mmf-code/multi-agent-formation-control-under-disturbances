#include "agent_control_pkg/controllers/fuzzy_gt2_adapter.hpp"
#include <algorithm>
#include <cmath>

namespace agent_control_pkg::controllers {

FuzzyGT2Adapter::FuzzyGT2Adapter(const Options &opt) : opt_(opt) {
  // No sets/rules yet; must call configureFromParams before use if using external params.
}

void FuzzyGT2Adapter::configureFromParams(const std::map<std::string, std::map<std::string, agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU>> &sets,
                           const std::vector<std::array<std::string,4>> &rules,
                           bool include_wind) {
  include_wind_ = include_wind;
  // Define variables present in sets map
  if (sets.count("error")) fls_.addInputVariable("error");
  if (sets.count("dError")) fls_.addInputVariable("dError");
  if (include_wind && sets.count("wind")) fls_.addInputVariable("wind");
  fls_.addOutputVariable("output");

  for (const auto &var_kv : sets) {
    const auto &var = var_kv.first;
    for (const auto &set_kv : var_kv.second) {
      fls_.addFuzzySetToVariable(var, set_kv.first, set_kv.second);
    }
  }

  for (size_t i = 0; i < rules.size(); ++i) {
    GT2FuzzyLogicSystem::FuzzyRule r;
    // Expected order: [error_set, dError_set, wind_set, output_set]
    if (sets.count("error")) r.antecedents["error"] = rules[i][0];
    if (sets.count("dError")) r.antecedents["dError"] = rules[i][1];
    if (include_wind && sets.count("wind")) r.antecedents["wind"] = rules[i][2];
    r.consequent = std::make_pair(std::string("output"), std::string(rules[i][3]));
    fls_.addRule(r);
  }

  initialized_ = true;
}

void FuzzyGT2Adapter::configureFromFuzzyParams(const agent_control_pkg::FuzzyParams &fp,
                                bool include_wind) {
  include_wind_ = include_wind;
  // Determine variable roles
  const bool has_error  = fp.sets.find("error")  != fp.sets.end();
  const bool has_derror = fp.sets.find("dError") != fp.sets.end();
  const bool has_wind   = include_wind && (fp.sets.find("wind") != fp.sets.end());
  if (has_error)  fls_.addInputVariable("error");
  if (has_derror) fls_.addInputVariable("dError");
  if (has_wind)   fls_.addInputVariable("wind");

  // Pick output variable name: prefer 'output' if present, else first non-input key
  std::string out_var = "output";
  if (fp.sets.find(out_var) == fp.sets.end()) {
    for (const auto &kv : fp.sets) {
      if (kv.first != "error" && kv.first != "dError" && kv.first != "wind") { out_var = kv.first; break; }
    }
  }
  fls_.addOutputVariable(out_var);

  // Add all sets (mapped to GT2FLS FOU struct)
  for (const auto &vk : fp.sets) {
    for (const auto &sk : vk.second) {
      agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU fou{sk.second.l1, sk.second.l2, sk.second.l3, sk.second.u1, sk.second.u2, sk.second.u3};
      fls_.addFuzzySetToVariable(vk.first, sk.first, fou);
    }
  }

  // Add rules with detected output variable
  for (size_t i = 0; i < fp.rules.size(); ++i) {
    GT2FuzzyLogicSystem::FuzzyRule r;
    if (has_error)  r.antecedents["error"]  = fp.rules[i][0];
    if (has_derror) r.antecedents["dError"] = fp.rules[i][1];
    if (has_wind)   r.antecedents["wind"]   = fp.rules[i][2];
    r.consequent = std::make_pair(out_var, std::string(fp.rules[i][3]));
    fls_.addRule(r);
  }
  initialized_ = true;
}

double FuzzyGT2Adapter::compute(double y, double yref, double dt) {
  if (dt <= 0.0) return 0.0;
  const double e = yref - y;
  double de_dt = 0.0;
  if (!first_) {
    de_dt = (e - prev_error_) / dt;
  } else {
    first_ = false;
  }
  prev_error_ = e;

  double u = 0.0;
  if (initialized_) {
    const double wind = include_wind_ ? opt_.wind_scalar : 0.0;
    u = fls_.calculateOutput(e, de_dt, wind);
  } else {
    // If not configured, fall back to a simple proportional action as a safe default
    u = e;
  }
  // Clamp
  if (u < opt_.umin) u = opt_.umin;
  if (u > opt_.umax) u = opt_.umax;
  return u;
}

void FuzzyGT2Adapter::reset() {
  prev_error_ = 0.0;
  first_ = true;
}

} // namespace agent_control_pkg::controllers
