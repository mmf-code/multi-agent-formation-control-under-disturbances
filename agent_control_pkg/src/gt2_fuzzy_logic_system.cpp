/**
 * @file gt2_fuzzy_logic_system.cpp
 * @brief Implementation of General Type-2 Fuzzy Logic System using Alpha-Plane Method
 *
 * Alpha-Plane (zSlices) Approach:
 * 1. Discretize secondary MF into N α-levels
 * 2. At each α-level, compute IT2-like type reduction
 * 3. Combine results using α-weighted average
 *
 * Computational Complexity: O(N × IT2_complexity)
 * where N = number of alpha levels
 *
 * @author Multi-Agent Formation Control Research
 * @date 2026-01-03
 */

#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>

// #define DEBUG_GT2  // Uncomment for debug output

#ifdef DEBUG_GT2
#define GT2_LOG(x) std::cout << "[GT2] " << x
#else
#define GT2_LOG(x)
#endif

namespace agent_control_pkg {

// ========== Secondary Membership Calculation ==========

double GT2FuzzyLogicSystem::GT2TriangularFS::getSecondaryMembership(
    double u, double lower, double upper) const
{
    if (upper - lower < 1e-9) {
        // Degenerate case: single point
        return 1.0;
    }

    // Normalize u to [0, 1] within the FOU interval
    double normalized_u = (u - lower) / (upper - lower);
    normalized_u = std::max(0.0, std::min(1.0, normalized_u));

    switch (secondary_shape) {
        case SecondaryMFShape::TRIANGULAR: {
            // Triangle peaked at secondary_peak_location (default: 0.5 = center)
            double peak_pos = secondary_peak_location;
            if (normalized_u <= peak_pos) {
                return normalized_u / peak_pos;
            } else {
                return (1.0 - normalized_u) / (1.0 - peak_pos);
            }
        }

        case SecondaryMFShape::GAUSSIAN: {
            // Gaussian centered at secondary_peak_location
            double center = secondary_peak_location;
            double sigma = secondary_spread;
            double exponent = -0.5 * std::pow((normalized_u - center) / sigma, 2);
            return std::exp(exponent);
        }

        case SecondaryMFShape::TRAPEZOIDAL: {
            // Trapezoid: flat top from 0.25 to 0.75
            if (normalized_u < 0.25) {
                return normalized_u / 0.25;
            } else if (normalized_u <= 0.75) {
                return 1.0;
            } else {
                return (1.0 - normalized_u) / 0.25;
            }
        }

        case SecondaryMFShape::UNIFORM:
        default:
            // Uniform = constant 1 (reduces to IT2)
            return 1.0;
    }
}

// ========== Constructor/Destructor ==========

GT2FuzzyLogicSystem::GT2FuzzyLogicSystem()
    : config_()
    , last_type_reduced_interval_{0.0, 0.0}
{
    GT2_LOG("GT2FuzzyLogicSystem: Default constructor" << std::endl);
}

GT2FuzzyLogicSystem::GT2FuzzyLogicSystem(const GT2Config& config)
    : config_(config)
    , last_type_reduced_interval_{0.0, 0.0}
{
    GT2_LOG("GT2FuzzyLogicSystem: Config constructor with "
            << config.num_alpha_levels << " alpha levels" << std::endl);
}

GT2FuzzyLogicSystem::~GT2FuzzyLogicSystem() {}

// ========== Configuration Methods ==========

void GT2FuzzyLogicSystem::setConfig(const GT2Config& config) {
    config_ = config;
    GT2_LOG("Config updated: " << config_.num_alpha_levels << " alpha levels" << std::endl);
}

void GT2FuzzyLogicSystem::addInputVariable(const std::string& var_name) {
    if (fuzzy_sets_.find(var_name) == fuzzy_sets_.end()) {
        fuzzy_sets_[var_name] = {};
        GT2_LOG("Added input variable: " << var_name << std::endl);
    }
}

void GT2FuzzyLogicSystem::addOutputVariable(const std::string& var_name) {
    if (fuzzy_sets_.find(var_name) == fuzzy_sets_.end()) {
        fuzzy_sets_[var_name] = {};
        output_variable_name_ = var_name;
        GT2_LOG("Added output variable: " << var_name << std::endl);
    }
}

void GT2FuzzyLogicSystem::addFuzzySetToVariable(
    const std::string& var_name,
    const std::string& set_name,
    const GT2TriangularFS& fs)
{
    if (fuzzy_sets_.count(var_name) > 0) {
        fuzzy_sets_[var_name][set_name] = fs;
        GT2_LOG("Added set '" << set_name << "' to '" << var_name << "'" << std::endl);
    } else {
        std::cerr << "GT2-FLS Error: Variable '" << var_name
                  << "' not defined before adding set." << std::endl;
    }
}

void GT2FuzzyLogicSystem::addRule(const FuzzyRule& rule) {
    rules_.push_back(rule);
}

// ========== Alpha Level Generation ==========

std::vector<double> GT2FuzzyLogicSystem::generateAlphaLevels() const {
    std::vector<double> levels;

    if (!config_.use_uniform_alpha && !config_.alpha_values.empty()) {
        // Use custom alpha values
        levels = config_.alpha_values;
    } else {
        // Generate uniform alpha levels
        // α ∈ (0, 1] - we exclude 0 because it corresponds to full FOU
        int n = config_.num_alpha_levels;
        for (int i = 1; i <= n; ++i) {
            double alpha = static_cast<double>(i) / static_cast<double>(n);
            levels.push_back(alpha);
        }
    }

    // Sort descending (process from α=1 down to α→0)
    std::sort(levels.begin(), levels.end(), std::greater<double>());
    return levels;
}

std::vector<double> GT2FuzzyLogicSystem::getAlphaLevels() const {
    return generateAlphaLevels();
}

// ========== Alpha-Cut Membership ==========

GT2FuzzyLogicSystem::MembershipInterval GT2FuzzyLogicSystem::getAlphaCutMembership(
    double crisp_input,
    const GT2TriangularFS& fs,
    double alpha_level)
{
    // First, compute full FOU membership (as in IT2)
    auto computeTriangularMembership = [](double x, double left, double peak, double right) -> double {
        if (x < left || x > right) return 0.0;
        if (peak - left > 1e-9 && x <= peak) {
            return (x - left) / (peak - left);
        }
        if (right - peak > 1e-9 && x > peak) {
            return (right - x) / (right - peak);
        }
        if (std::abs(x - peak) < 1e-9) {
            return 1.0;
        }
        return 0.0;
    };

    double lower_mu = computeTriangularMembership(
        crisp_input, fs.lmf_left_base, fs.lmf_peak, fs.lmf_right_base);
    double upper_mu = computeTriangularMembership(
        crisp_input, fs.umf_left_base, fs.umf_peak, fs.umf_right_base);

    // Ensure lower <= upper
    if (lower_mu > upper_mu) {
        std::swap(lower_mu, upper_mu);
    }

    // Clamp to [0, 1]
    lower_mu = std::max(0.0, std::min(1.0, lower_mu));
    upper_mu = std::max(0.0, std::min(1.0, upper_mu));

    // For GT2: shrink the interval based on α-cut of secondary MF
    // At α-cut level α:
    // - Find u values where secondary_MF(u) >= α
    // - These form the α-cut interval [u_lower_α, u_upper_α]

    if (upper_mu - lower_mu < 1e-9) {
        // Degenerate FOU - just return the point
        return {lower_mu, upper_mu};
    }

    // For secondary MF shapes, find where secondary grade >= alpha
    // This depends on the shape:
    double alpha_lower = lower_mu;
    double alpha_upper = upper_mu;

    switch (fs.secondary_shape) {
        case SecondaryMFShape::TRIANGULAR: {
            // Triangle with peak at center: μ_sec(u) = 1 - 2*|u - 0.5|
            // At α-cut: u ∈ [0.5 - (1-α)/2, 0.5 + (1-α)/2]
            double half_width = (1.0 - alpha_level) / 2.0;
            double center = fs.secondary_peak_location;
            double u_lower_norm = center - half_width * center;
            double u_upper_norm = center + half_width * (1.0 - center);
            u_lower_norm = std::max(0.0, std::min(1.0, u_lower_norm));
            u_upper_norm = std::max(0.0, std::min(1.0, u_upper_norm));

            // Map back to actual membership interval
            double interval_width = upper_mu - lower_mu;
            alpha_lower = lower_mu + u_lower_norm * interval_width;
            alpha_upper = lower_mu + u_upper_norm * interval_width;
            break;
        }

        case SecondaryMFShape::GAUSSIAN: {
            // Gaussian: find points where exp(-((u-c)^2)/(2σ^2)) >= α
            // This gives: |u - c| <= σ * sqrt(-2 * ln(α))
            double sigma = fs.secondary_spread;
            if (alpha_level > 1e-9 && alpha_level < 1.0) {
                double radius = sigma * std::sqrt(-2.0 * std::log(alpha_level));
                double center = fs.secondary_peak_location;
                double u_lower_norm = std::max(0.0, center - radius);
                double u_upper_norm = std::min(1.0, center + radius);

                double interval_width = upper_mu - lower_mu;
                alpha_lower = lower_mu + u_lower_norm * interval_width;
                alpha_upper = lower_mu + u_upper_norm * interval_width;
            }
            break;
        }

        case SecondaryMFShape::TRAPEZOIDAL: {
            // Trapezoid: flat from 0.25 to 0.75
            // At α-cut: if α <= 1, always includes [0.25, 0.75]
            // For α < 1: extends to α*0.25 and 1 - α*0.25
            double u_lower_norm = alpha_level * 0.25;
            double u_upper_norm = 1.0 - alpha_level * 0.25;

            double interval_width = upper_mu - lower_mu;
            alpha_lower = lower_mu + u_lower_norm * interval_width;
            alpha_upper = lower_mu + u_upper_norm * interval_width;
            break;
        }

        case SecondaryMFShape::UNIFORM:
        default:
            // Uniform: α-cut is always the full interval (reduces to IT2)
            alpha_lower = lower_mu;
            alpha_upper = upper_mu;
            break;
    }

    // Ensure valid interval
    if (alpha_lower > alpha_upper) {
        std::swap(alpha_lower, alpha_upper);
    }

    return {alpha_lower, alpha_upper};
}

// ========== Fuzzification at Alpha Level ==========

std::map<std::string, std::map<std::string, GT2FuzzyLogicSystem::MembershipInterval>>
GT2FuzzyLogicSystem::fuzzifyInputsAtAlpha(
    double alpha_level,
    double crisp_error,
    double crisp_dError,
    double crisp_wind)
{
    std::map<std::string, std::map<std::string, MembershipInterval>> result;

    // Map inputs to variable names
    std::map<std::string, double> inputs = {
        {"error", crisp_error},
        {"dError", crisp_dError},
        {"wind", crisp_wind}
    };

    for (const auto& var_pair : fuzzy_sets_) {
        const std::string& var_name = var_pair.first;
        const auto& sets = var_pair.second;

        // Skip output variable during fuzzification
        if (var_name == output_variable_name_) continue;

        // Find the input value for this variable
        auto input_it = inputs.find(var_name);
        if (input_it == inputs.end()) continue;

        double crisp_value = input_it->second;

        for (const auto& set_pair : sets) {
            const std::string& set_name = set_pair.first;
            const GT2TriangularFS& fs = set_pair.second;

            MembershipInterval membership = getAlphaCutMembership(
                crisp_value, fs, alpha_level);

            result[var_name][set_name] = membership;
        }
    }

    return result;
}

// ========== Rule Inference ==========

std::vector<GT2FuzzyLogicSystem::RuleFiringInfo>
GT2FuzzyLogicSystem::calculateRuleFirings(
    const std::map<std::string, std::map<std::string, MembershipInterval>>& fuzzified_inputs)
{
    std::vector<RuleFiringInfo> fired_rules;

    if (rules_.empty() || output_variable_name_.empty()) {
        return fired_rules;
    }

    for (size_t rule_idx = 0; rule_idx < rules_.size(); ++rule_idx) {
        const FuzzyRule& rule = rules_[rule_idx];

        // Min t-norm: start with [1, 1]
        MembershipInterval firing_strength = {1.0, 1.0};
        bool rule_valid = true;

        for (const auto& antecedent : rule.antecedents) {
            const std::string& var_name = antecedent.first;
            const std::string& set_name = antecedent.second;

            auto var_it = fuzzified_inputs.find(var_name);
            if (var_it == fuzzified_inputs.end()) {
                firing_strength = {0.0, 0.0};
                rule_valid = false;
                break;
            }

            auto set_it = var_it->second.find(set_name);
            if (set_it == var_it->second.end()) {
                firing_strength = {0.0, 0.0};
                rule_valid = false;
                break;
            }

            const MembershipInterval& membership = set_it->second;
            firing_strength.first = std::min(firing_strength.first, membership.first);
            firing_strength.second = std::min(firing_strength.second, membership.second);
        }

        // Only add if rule fires
        if (rule_valid && firing_strength.second > 1e-9) {
            const std::string& output_set = rule.consequent.second;

            if (fuzzy_sets_.count(output_variable_name_) > 0 &&
                fuzzy_sets_.at(output_variable_name_).count(output_set) > 0) {

                double centroid = fuzzy_sets_.at(output_variable_name_).at(output_set).getCentroid();
                fired_rules.push_back({firing_strength, centroid, rule_idx});
            }
        }
    }

    return fired_rules;
}

// ========== Karnik-Mendel Type Reduction ==========

std::pair<double, double> GT2FuzzyLogicSystem::typeReduce_KarnikMendel(
    std::vector<RuleFiringInfo>& rule_firings)
{
    if (rule_firings.empty()) {
        return {0.0, 0.0};
    }

    // Sort by consequent centroid
    std::sort(rule_firings.begin(), rule_firings.end(),
              [](const RuleFiringInfo& a, const RuleFiringInfo& b) {
                  return a.consequent_centroid < b.consequent_centroid;
              });

    int N = static_cast<int>(rule_firings.size());

    // === Calculate y_l (left endpoint) ===
    double y_l_prime_num = 0.0, y_l_prime_den = 0.0;
    for (const auto& info : rule_firings) {
        y_l_prime_num += info.firing_strength.first * info.consequent_centroid;
        y_l_prime_den += info.firing_strength.first;
    }
    double y_l_prime = (y_l_prime_den < 1e-9) ? 0.0 : y_l_prime_num / y_l_prime_den;

    double y_l_prev = std::numeric_limits<double>::lowest();
    int max_iter = 20, iter = 0;

    while (std::abs(y_l_prime - y_l_prev) > 1e-6 && iter < max_iter) {
        y_l_prev = y_l_prime;

        int k = 0;
        while (k < N - 1 && rule_firings[k + 1].consequent_centroid <= y_l_prime) {
            ++k;
        }

        double num = 0.0, den = 0.0;
        for (int i = 0; i <= k; ++i) {
            num += rule_firings[i].firing_strength.second * rule_firings[i].consequent_centroid;
            den += rule_firings[i].firing_strength.second;
        }
        for (int i = k + 1; i < N; ++i) {
            num += rule_firings[i].firing_strength.first * rule_firings[i].consequent_centroid;
            den += rule_firings[i].firing_strength.first;
        }

        y_l_prime = (den < 1e-9) ? 0.0 : num / den;
        ++iter;
    }

    // === Calculate y_r (right endpoint) ===
    double y_r_prime_num = 0.0, y_r_prime_den = 0.0;
    for (const auto& info : rule_firings) {
        y_r_prime_num += info.firing_strength.second * info.consequent_centroid;
        y_r_prime_den += info.firing_strength.second;
    }
    double y_r_prime = (y_r_prime_den < 1e-9) ? 0.0 : y_r_prime_num / y_r_prime_den;

    double y_r_prev = std::numeric_limits<double>::lowest();
    iter = 0;

    while (std::abs(y_r_prime - y_r_prev) > 1e-6 && iter < max_iter) {
        y_r_prev = y_r_prime;

        int k = 0;
        while (k < N - 1 && rule_firings[k + 1].consequent_centroid <= y_r_prime) {
            ++k;
        }

        double num = 0.0, den = 0.0;
        for (int i = 0; i <= k; ++i) {
            num += rule_firings[i].firing_strength.first * rule_firings[i].consequent_centroid;
            den += rule_firings[i].firing_strength.first;
        }
        for (int i = k + 1; i < N; ++i) {
            num += rule_firings[i].firing_strength.second * rule_firings[i].consequent_centroid;
            den += rule_firings[i].firing_strength.second;
        }

        y_r_prime = (den < 1e-9) ? 0.0 : num / den;
        ++iter;
    }

    // Ensure y_l <= y_r
    if (y_l_prime > y_r_prime) {
        std::swap(y_l_prime, y_r_prime);
    }

    return {y_l_prime, y_r_prime};
}

// ========== Process Single Alpha Plane ==========

GT2FuzzyLogicSystem::AlphaPlaneResult GT2FuzzyLogicSystem::processAlphaPlane(
    double alpha_level,
    double crisp_error,
    double crisp_dError,
    double crisp_wind)
{
    GT2_LOG("Processing alpha-plane at α=" << alpha_level << std::endl);

    // 1. Fuzzify at this alpha level
    auto fuzzified = fuzzifyInputsAtAlpha(alpha_level, crisp_error, crisp_dError, crisp_wind);

    // 2. Calculate rule firings
    auto rule_firings = calculateRuleFirings(fuzzified);

    // 3. Type reduction (KM algorithm)
    auto [y_l, y_r] = typeReduce_KarnikMendel(rule_firings);

    // 4. Calculate weight for this alpha level
    double weight = calculateSecondaryWeight(alpha_level);

    AlphaPlaneResult result;
    result.alpha_level = alpha_level;
    result.y_left = y_l;
    result.y_right = y_r;
    result.y_centroid = (y_l + y_r) / 2.0;
    result.weight = weight;

    GT2_LOG("  α=" << alpha_level << ": y=[" << y_l << ", " << y_r
            << "], centroid=" << result.y_centroid << ", weight=" << weight << std::endl);

    return result;
}

// ========== Secondary Weight Calculation ==========

double GT2FuzzyLogicSystem::calculateSecondaryWeight(double alpha_level) const {
    // For alpha-plane method:
    // Weight = α_i (linear weighting)
    // Alternative: Weight = α_i² (quadratic, emphasizes higher α)
    // Alternative: Weight = 1 (uniform)

    // Use linear weighting by default
    return alpha_level;
}

// ========== Combine Alpha Plane Results ==========

double GT2FuzzyLogicSystem::combineAlphaPlaneResults(
    const std::vector<AlphaPlaneResult>& results)
{
    if (results.empty()) {
        return 0.0;
    }

    // Weighted average of centroids
    // y_final = Σ(weight_i × centroid_i) / Σ(weight_i)

    double weighted_sum = 0.0;
    double weight_sum = 0.0;

    for (const auto& result : results) {
        weighted_sum += result.weight * result.y_centroid;
        weight_sum += result.weight;
    }

    if (weight_sum < 1e-9) {
        return 0.0;
    }

    double final_output = weighted_sum / weight_sum;

    GT2_LOG("Combined output from " << results.size()
            << " alpha-planes: " << final_output << std::endl);

    return final_output;
}

// ========== Main Calculation ==========

double GT2FuzzyLogicSystem::calculateOutput(
    double crisp_error,
    double crisp_dError,
    double crisp_wind)
{
    GT2_LOG("\n=== GT2-FLS Calculation Start ===" << std::endl);
    GT2_LOG("Inputs: error=" << crisp_error << ", dError=" << crisp_dError
            << ", wind=" << crisp_wind << std::endl);

    // Clear previous results
    last_alpha_results_.clear();

    // Generate alpha levels
    std::vector<double> alpha_levels = generateAlphaLevels();

    GT2_LOG("Processing " << alpha_levels.size() << " alpha-planes" << std::endl);

    // Process each alpha-plane
    for (double alpha : alpha_levels) {
        AlphaPlaneResult result = processAlphaPlane(
            alpha, crisp_error, crisp_dError, crisp_wind);
        last_alpha_results_.push_back(result);
    }

    // Combine results
    double final_output = combineAlphaPlaneResults(last_alpha_results_);

    // Store type-reduced interval (using first alpha-plane as reference)
    if (!last_alpha_results_.empty()) {
        // Compute overall bounds
        double y_l_min = last_alpha_results_[0].y_left;
        double y_r_max = last_alpha_results_[0].y_right;
        for (const auto& r : last_alpha_results_) {
            y_l_min = std::min(y_l_min, r.y_left);
            y_r_max = std::max(y_r_max, r.y_right);
        }
        last_type_reduced_interval_ = {y_l_min, y_r_max};
    }

    GT2_LOG("=== GT2-FLS Final Output: " << final_output << " ===" << std::endl);

    return final_output;
}

} // namespace agent_control_pkg
