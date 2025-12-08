/**
 * @file it2_fuzzy_logic_system.cpp
 * @brief Implementation of Interval Type-2 Fuzzy Logic System (IT2-FLS)
 *
 * This implements the IT2-FLS pipeline:
 * 1. Fuzzification using IT2 triangular MFs with FOU
 * 2. Rule inference with min t-norm
 * 3. Karnik-Mendel type reduction
 * 4. Centroid defuzzification
 */

#include "agent_control_pkg/it2_fuzzy_logic_system.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <vector>
#include <iostream>
#include <limits>

// Conditional debug logging
#ifdef DEBUG_FLS
#define FLS_LOG(x) std::cout << x
#else
#define FLS_LOG(x)
#endif

namespace agent_control_pkg {

// Constructor
IT2FuzzyLogicSystem::IT2FuzzyLogicSystem() {
  FLS_LOG("IT2FuzzyLogicSystem: Constructor." << std::endl);
}

// Destructor
IT2FuzzyLogicSystem::~IT2FuzzyLogicSystem() {}

// --- Configuration Methods ---

void IT2FuzzyLogicSystem::addInputVariable(const std::string &var_name) {
  if (fuzzy_sets_.find(var_name) == fuzzy_sets_.end()) {
    fuzzy_sets_[var_name] = {};
    FLS_LOG("IT2-FLS: Added input variable: " << var_name << std::endl);
  }
}

void IT2FuzzyLogicSystem::addOutputVariable(const std::string &var_name) {
  if (fuzzy_sets_.find(var_name) == fuzzy_sets_.end()) {
    fuzzy_sets_[var_name] = {};
    output_variable_name_ = var_name;
    FLS_LOG("IT2-FLS: Added output variable: " << var_name << std::endl);
  }
}

void IT2FuzzyLogicSystem::addFuzzySetToVariable(
    const std::string &var_name, const std::string &set_name,
    const IT2TriangularFS_FOU &fou) {
  if (fuzzy_sets_.count(var_name) > 0) {
    fuzzy_sets_[var_name][set_name] = fou;
    FLS_LOG("IT2-FLS: Added set '" << set_name << "' to variable '" << var_name
                                   << "'" << std::endl);
  } else {
    std::cerr << "IT2-FLS: Error - Variable '" << var_name
              << "' not defined before adding set." << std::endl;
  }
}

void IT2FuzzyLogicSystem::addRule(const FuzzyRule &rule) {
  rules_.push_back(rule);
}

// --- Membership Calculation ---

IT2FuzzyLogicSystem::MembershipInterval
IT2FuzzyLogicSystem::getTriangularMembership(double crisp_input, double l_left,
                                             double l_peak, double l_right,
                                             double u_left, double u_peak,
                                             double u_right) {
  double lower_mu = 0.0;
  double upper_mu = 0.0;

  // Lower Membership Function (Triangle)
  if (l_peak - l_left > 1e-9) {
    if (crisp_input >= l_left && crisp_input <= l_peak) {
      lower_mu = (crisp_input - l_left) / (l_peak - l_left);
    }
  } else if (std::abs(crisp_input - l_peak) < 1e-9) {
    lower_mu = 1.0;
  }
  if (l_right - l_peak > 1e-9) {
    if (crisp_input > l_peak && crisp_input <= l_right) {
      lower_mu = (l_right - crisp_input) / (l_right - l_peak);
    }
  } else if (std::abs(crisp_input - l_peak) < 1e-9 &&
             !(l_peak - l_left > 1e-9)) {
    lower_mu = std::max(lower_mu, 1.0);
  }

  // Upper Membership Function (Triangle)
  if (u_peak - u_left > 1e-9) {
    if (crisp_input >= u_left && crisp_input <= u_peak) {
      upper_mu = (crisp_input - u_left) / (u_peak - u_left);
    }
  } else if (std::abs(crisp_input - u_peak) < 1e-9) {
    upper_mu = 1.0;
  }
  if (u_right - u_peak > 1e-9) {
    if (crisp_input > u_peak && crisp_input <= u_right) {
      upper_mu = (u_right - crisp_input) / (u_right - u_peak);
    }
  } else if (std::abs(crisp_input - u_peak) < 1e-9 &&
             !(u_peak - u_left > 1e-9)) {
    upper_mu = std::max(upper_mu, 1.0);
  }

  // Ensure lower <= upper membership and bounds [0, 1]
  lower_mu = std::min(lower_mu, upper_mu);
  lower_mu = std::max(0.0, std::min(lower_mu, 1.0));
  upper_mu = std::max(0.0, std::min(upper_mu, 1.0));

  return {lower_mu, upper_mu};
}

// --- Fuzzification ---

std::map<std::string,
         std::map<std::string, IT2FuzzyLogicSystem::MembershipInterval>>
IT2FuzzyLogicSystem::fuzzifyInputs(double crisp_error, double crisp_dError,
                                   double crisp_wind) {
  FLS_LOG("IT2-FLS: Fuzzifying Inputs - Error="
          << crisp_error << ", dError=" << crisp_dError
          << ", Wind=" << crisp_wind << std::endl);

  std::map<std::string, MembershipInterval> error_memberships;
  std::map<std::string, MembershipInterval> dError_memberships;
  std::map<std::string, MembershipInterval> wind_memberships;

  if (fuzzy_sets_.count("error") > 0) {
    FLS_LOG("  Fuzzifying error input: " << crisp_error << std::endl);
    for (const auto &pair : fuzzy_sets_.at("error")) {
      const std::string &set_name = pair.first;
      const IT2TriangularFS_FOU &fou = pair.second;
      error_memberships[set_name] = getTriangularMembership(
          crisp_error, fou.lmf_left_base, fou.lmf_peak, fou.lmf_right_base,
          fou.umf_left_base, fou.umf_peak, fou.umf_right_base);
      FLS_LOG("    Error Set '"
              << set_name << "': [" << error_memberships[set_name].first << ", "
              << error_memberships[set_name].second << "]" << std::endl);
    }
  }

  if (fuzzy_sets_.count("dError") > 0) {
    FLS_LOG("  Fuzzifying dError input: " << crisp_dError << std::endl);
    for (const auto &pair : fuzzy_sets_.at("dError")) {
      const std::string &set_name = pair.first;
      const IT2TriangularFS_FOU &fou = pair.second;
      dError_memberships[set_name] = getTriangularMembership(
          crisp_dError, fou.lmf_left_base, fou.lmf_peak, fou.lmf_right_base,
          fou.umf_left_base, fou.umf_peak, fou.umf_right_base);
      FLS_LOG("    dError Set '" << set_name << "': ["
                                 << dError_memberships[set_name].first << ", "
                                 << dError_memberships[set_name].second << "]"
                                 << std::endl);
    }
  }

  if (fuzzy_sets_.count("wind") > 0) {
    FLS_LOG("  Fuzzifying wind input: " << crisp_wind << std::endl);
    for (const auto &pair : fuzzy_sets_.at("wind")) {
      const std::string &set_name = pair.first;
      const IT2TriangularFS_FOU &fou = pair.second;
      wind_memberships[set_name] = getTriangularMembership(
          crisp_wind, fou.lmf_left_base, fou.lmf_peak, fou.lmf_right_base,
          fou.umf_left_base, fou.umf_peak, fou.umf_right_base);
      FLS_LOG("    Wind Set '"
              << set_name << "': [" << wind_memberships[set_name].first << ", "
              << wind_memberships[set_name].second << "]" << std::endl);
    }
  }

  std::map<std::string, std::map<std::string, MembershipInterval>>
      all_fuzzified_inputs;
  if (!error_memberships.empty())
    all_fuzzified_inputs["error"] = error_memberships;
  if (!dError_memberships.empty())
    all_fuzzified_inputs["dError"] = dError_memberships;
  if (!wind_memberships.empty())
    all_fuzzified_inputs["wind"] = wind_memberships;

  return all_fuzzified_inputs;
}

// --- Rule Inference ---

std::vector<IT2FuzzyLogicSystem::RuleFiringInfo>
IT2FuzzyLogicSystem::calculateRuleFirings(
    const std::map<std::string, std::map<std::string, MembershipInterval>>
        &fuzzified_inputs) {
  FLS_LOG("IT2-FLS: Calculating Rule Firings for " << rules_.size()
                                                   << " rules..." << std::endl);
  std::vector<RuleFiringInfo> fired_rules_info;

  if (rules_.empty()) {
    std::cerr << "IT2-FLS Warning: No rules defined!" << std::endl;
    return fired_rules_info;
  }
  if (output_variable_name_.empty() ||
      fuzzy_sets_.count(output_variable_name_) == 0) {
    std::cerr
        << "IT2-FLS Error: Output variable or its sets not defined correctly."
        << std::endl;
    return fired_rules_info;
  }

  for (size_t rule_idx = 0; rule_idx < rules_.size(); ++rule_idx) {
    const FuzzyRule &rule = rules_[rule_idx];

    // Min t-norm: start with 1.0
    MembershipInterval rule_firing_strength = {1.0, 1.0};
    bool can_evaluate_this_rule = true;

    // Calculate antecedent firing strength using min t-norm
    for (const auto &antecedent_pair : rule.antecedents) {
      const std::string &input_var_name = antecedent_pair.first;
      const std::string &fuzzy_set_name = antecedent_pair.second;

      auto it_var = fuzzified_inputs.find(input_var_name);
      if (it_var != fuzzified_inputs.end()) {
        const auto &var_fuzz_sets = it_var->second;
        auto it_set = var_fuzz_sets.find(fuzzy_set_name);
        if (it_set != var_fuzz_sets.end()) {
          const MembershipInterval &term_membership = it_set->second;
          rule_firing_strength.first =
              std::min(rule_firing_strength.first, term_membership.first);
          rule_firing_strength.second =
              std::min(rule_firing_strength.second, term_membership.second);
        } else {
          rule_firing_strength = {0.0, 0.0};
          can_evaluate_this_rule = false;
          break;
        }
      } else {
        rule_firing_strength = {0.0, 0.0};
        can_evaluate_this_rule = false;
        break;
      }
    }

    // If rule fires (upper bound > 0), get consequent centroid
    if (can_evaluate_this_rule && rule_firing_strength.second > 1e-9) {
      const std::string &output_set_name = rule.consequent.second;

      if (fuzzy_sets_.at(output_variable_name_).count(output_set_name) > 0) {
        const IT2TriangularFS_FOU &consequent_fou =
            fuzzy_sets_.at(output_variable_name_).at(output_set_name);
        double consequent_centroid = consequent_fou.getLMFCentroid();

        fired_rules_info.push_back(
            {rule_firing_strength, consequent_centroid, rule_idx});
        FLS_LOG("  Rule " << rule_idx << " fired: Strength=["
                          << rule_firing_strength.first << ","
                          << rule_firing_strength.second << "], Consequent '"
                          << output_set_name << "' (centroid="
                          << consequent_centroid << ")" << std::endl);
      } else {
        std::cerr << "IT2-FLS Error: Consequent fuzzy set '" << output_set_name
                  << "' not found for output variable '"
                  << output_variable_name_ << "' in rule " << rule_idx
                  << std::endl;
      }
    } else {
      FLS_LOG("  Rule " << rule_idx << " did not fire significantly."
                        << std::endl);
    }
  }
  return fired_rules_info;
}

// --- Karnik-Mendel Type Reduction ---

std::pair<double, double> IT2FuzzyLogicSystem::typeReduce_KarnikMendel(
    std::vector<RuleFiringInfo> &rule_firings_info) {
  FLS_LOG("IT2-FLS: Type Reducing using Karnik-Mendel for "
          << rule_firings_info.size() << " fired rules." << std::endl);

  if (rule_firings_info.empty()) {
    FLS_LOG("  No rules fired, KM returning [0,0]." << std::endl);
    return {0.0, 0.0};
  }

  // Sort by consequent centroid (required for KM algorithm)
  std::sort(rule_firings_info.begin(), rule_firings_info.end(),
            [](const RuleFiringInfo &a, const RuleFiringInfo &b) {
              return a.consequent_centroid < b.consequent_centroid;
            });

#ifdef DEBUG_FLS
  FLS_LOG("  Sorted rule firings by consequent centroid:" << std::endl);
  for (const auto &info : rule_firings_info) {
    FLS_LOG("    y_i=" << info.consequent_centroid
                       << ", f_lower=" << info.firing_strength.first
                       << ", f_upper=" << info.firing_strength.second
                       << std::endl);
  }
#endif

  double y_l = 0.0;
  double y_r = 0.0;
  int N = static_cast<int>(rule_firings_info.size());

  // --- Calculate y_l (left endpoint) ---
  double y_l_prime_num = 0.0;
  double y_l_prime_den = 0.0;
  for (const auto &info : rule_firings_info) {
    y_l_prime_num += info.firing_strength.first * info.consequent_centroid;
    y_l_prime_den += info.firing_strength.first;
  }
  double y_l_prime =
      (y_l_prime_den == 0.0) ? 0.0 : y_l_prime_num / y_l_prime_den;

  double y_l_prev_prime = std::numeric_limits<double>::lowest();
  int max_iterations = 20;
  int iter_count = 0;

  while (std::abs(y_l_prime - y_l_prev_prime) > 1e-6 &&
         iter_count < max_iterations) {
    y_l_prev_prime = y_l_prime;

    int k_switch = 0;
    while (k_switch < N - 1 &&
           rule_firings_info[k_switch + 1].consequent_centroid <= y_l_prime) {
      ++k_switch;
    }

    double num = 0.0;
    double den = 0.0;
    for (int i = 0; i <= k_switch; ++i) {
      num += rule_firings_info[i].firing_strength.second *
             rule_firings_info[i].consequent_centroid;
      den += rule_firings_info[i].firing_strength.second;
    }
    for (int i = k_switch + 1; i < N; ++i) {
      num += rule_firings_info[i].firing_strength.first *
             rule_firings_info[i].consequent_centroid;
      den += rule_firings_info[i].firing_strength.first;
    }

    y_l_prime = (den == 0.0) ? 0.0 : num / den;
    iter_count++;
  }
  y_l = y_l_prime;
  FLS_LOG("  KM y_l converged to " << y_l << " in " << iter_count
                                   << " iterations." << std::endl);

  // --- Calculate y_r (right endpoint) ---
  double y_r_prime_num = 0.0;
  double y_r_prime_den = 0.0;
  for (const auto &info : rule_firings_info) {
    y_r_prime_num += info.firing_strength.second * info.consequent_centroid;
    y_r_prime_den += info.firing_strength.second;
  }
  double y_r_prime = (y_r_prime_den == 0) ? 0.0 : y_r_prime_num / y_r_prime_den;

  double y_r_prev_prime = std::numeric_limits<double>::lowest();
  iter_count = 0;

  while (std::abs(y_r_prime - y_r_prev_prime) > 1e-6 &&
         iter_count < max_iterations) {
    y_r_prev_prime = y_r_prime;

    int k_switch = 0;
    while (k_switch < N - 1 &&
           rule_firings_info[k_switch + 1].consequent_centroid <= y_r_prime) {
      ++k_switch;
    }

    double num = 0.0;
    double den = 0.0;
    for (int i = 0; i <= k_switch; ++i) {
      num += rule_firings_info[i].firing_strength.first *
             rule_firings_info[i].consequent_centroid;
      den += rule_firings_info[i].firing_strength.first;
    }
    for (int i = k_switch + 1; i < N; ++i) {
      num += rule_firings_info[i].firing_strength.second *
             rule_firings_info[i].consequent_centroid;
      den += rule_firings_info[i].firing_strength.second;
    }

    y_r_prime = (den == 0.0) ? 0.0 : num / den;
    iter_count++;
  }
  y_r = y_r_prime;
  FLS_LOG("  KM y_r converged to " << y_r << " in " << iter_count
                                   << " iterations." << std::endl);

  // Ensure y_l <= y_r
  if (y_l > y_r) {
    FLS_LOG("  Warning: KM y_l > y_r. Swapping." << std::endl);
    std::swap(y_l, y_r);
  }

  FLS_LOG("IT2-FLS: KM Type Reduced Interval: [" << y_l << ", " << y_r << "]"
                                                 << std::endl);
  return {y_l, y_r};
}

// --- Defuzzification ---

double IT2FuzzyLogicSystem::defuzzify(
    const std::pair<double, double> &type_reduced_interval) {
  FLS_LOG("IT2-FLS: Defuzzify (Averaging interval)" << std::endl);
  double crisp_output =
      (type_reduced_interval.first + type_reduced_interval.second) / 2.0;
  FLS_LOG("  Defuzzified output: " << crisp_output << std::endl);
  return crisp_output;
}

// --- Main Calculation Method ---

double IT2FuzzyLogicSystem::calculateOutput(double crisp_error,
                                            double crisp_dError,
                                            double crisp_wind) {
  FLS_LOG("\n--- IT2-FLS Calculation Start ---" << std::endl);

  // 1. Fuzzify inputs
  auto fuzzified_inputs = fuzzifyInputs(crisp_error, crisp_dError, crisp_wind);

  // 2. Calculate rule firings
  std::vector<RuleFiringInfo> fired_rules =
      calculateRuleFirings(fuzzified_inputs);

  // 3. Type Reduction (Karnik-Mendel)
  std::pair<double, double> type_reduced_interval =
      typeReduce_KarnikMendel(fired_rules);

  // 4. Defuzzification
  double crisp_output = defuzzify(type_reduced_interval);

  FLS_LOG("IT2-FLS Final Crisp Output: " << crisp_output << std::endl);
  FLS_LOG("--- IT2-FLS Calculation End ---\n" << std::endl);
  return crisp_output;
}

} // namespace agent_control_pkg
