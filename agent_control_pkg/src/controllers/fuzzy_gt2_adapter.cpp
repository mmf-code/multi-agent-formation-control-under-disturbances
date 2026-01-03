/**
 * @file fuzzy_gt2_adapter.cpp
 * @brief Implementation of FuzzyGT2Adapter
 *
 * Provides GT2 (General Type-2) fuzzy logic control with alpha-plane
 * decomposition for handling full secondary membership functions.
 *
 * @author Multi-Agent Formation Control Research
 * @date 2026-01-03
 */

#include "agent_control_pkg/controllers/fuzzy_gt2_adapter.hpp"
#include "agent_control_pkg/fuzzy_system_factory.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace agent_control_pkg::controllers {

FuzzyGT2Adapter::FuzzyGT2Adapter(const Options &opt)
    : opt_(opt)
{
    // Configure the GT2 system with specified alpha levels and secondary shape
    GT2FuzzyLogicSystem::GT2Config config;
    config.num_alpha_levels = opt_.num_alpha_levels;
    config.use_uniform_alpha = true;
    config.default_secondary_shape = opt_.secondary_shape;
    config.secondary_spread = opt_.secondary_spread;

    fls_.setConfig(config);
    reset();
}

void FuzzyGT2Adapter::configureFromParams(
    const std::map<std::string,
                   std::map<std::string, GT2FuzzyLogicSystem::GT2TriangularFS>> &sets,
    const std::vector<std::array<std::string, 4>> &rules,
    bool include_wind)
{
    // Reinitialize with current config
    GT2FuzzyLogicSystem::GT2Config config;
    config.num_alpha_levels = opt_.num_alpha_levels;
    config.use_uniform_alpha = true;
    config.default_secondary_shape = opt_.secondary_shape;
    config.secondary_spread = opt_.secondary_spread;

    fls_ = GT2FuzzyLogicSystem(config);
    include_wind_ = include_wind;

    // Detect output variable name from provided sets
    std::string output_var = "";
    if (sets.count("correction")) {
        output_var = "correction";
    } else if (sets.count("output")) {
        output_var = "output";
    } else {
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
            const auto &fs = set_entry.second;
            fls_.addFuzzySetToVariable(var_name, set_name, fs);
        }
    }

    // Add rules: [error_label, dError_label, wind_label, output_label]
    for (const auto &r : rules) {
        GT2FuzzyLogicSystem::FuzzyRule rule;
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

void FuzzyGT2Adapter::configureDefault(bool include_wind)
{
    include_wind_ = include_wind;

    // Use factory to get fully configured GT2 system
    auto adapter = FuzzySystemFactory::createGT2(
        opt_.num_alpha_levels, opt_.secondary_shape);

    // Copy the system from the factory-created adapter
    fls_ = *adapter->getSystem();

    initialized_ = true;
}

void FuzzyGT2Adapter::configureFromFuzzyParams(const FuzzyParams &fp, bool include_wind)
{
    include_wind_ = include_wind;

    // Reinitialize GT2 system with current config
    GT2FuzzyLogicSystem::GT2Config config;
    config.num_alpha_levels = opt_.num_alpha_levels;
    config.use_uniform_alpha = true;
    config.default_secondary_shape = opt_.secondary_shape;
    config.secondary_spread = opt_.secondary_spread;

    fls_ = GT2FuzzyLogicSystem(config);

    // Helper to convert FuzzySetFOU to GT2TriangularFS
    // FOU format: {l1, l2, l3, u1, u2, u3} -> {lmf_left, lmf_peak, lmf_right, umf_left, umf_peak, umf_right}
    auto fouToGT2 = [this](const FuzzySetFOU &fou) -> GT2FuzzyLogicSystem::GT2TriangularFS {
        GT2FuzzyLogicSystem::GT2TriangularFS fs;
        fs.lmf_left_base = fou.l1;
        fs.lmf_peak = fou.l2;
        fs.lmf_right_base = fou.l3;
        fs.umf_left_base = fou.u1;
        fs.umf_peak = fou.u2;
        fs.umf_right_base = fou.u3;
        fs.secondary_shape = opt_.secondary_shape;
        fs.secondary_spread = opt_.secondary_spread;
        fs.secondary_peak_location = 0.5;  // Center of FOU
        return fs;
    };

    // Detect output variable name
    std::string output_var = "output";
    for (const auto &kv : fp.sets) {
        if (kv.first != "error" && kv.first != "dError" && kv.first != "wind") {
            output_var = kv.first;
            break;
        }
    }

    // Declare variables
    fls_.addInputVariable("error");
    fls_.addInputVariable("dError");
    if (include_wind_) {
        fls_.addInputVariable("wind");
    }
    fls_.addOutputVariable(output_var);

    // Load fuzzy sets - convert FOU to GT2TriangularFS
    for (const auto &var_entry : fp.sets) {
        const std::string &var_name = var_entry.first;
        for (const auto &set_entry : var_entry.second) {
            const std::string &set_name = set_entry.first;
            const auto &fou = set_entry.second;
            GT2FuzzyLogicSystem::GT2TriangularFS gt2_fs = fouToGT2(fou);
            fls_.addFuzzySetToVariable(var_name, set_name, gt2_fs);
        }
    }

    // Add rules
    for (const auto &r : fp.rules) {
        GT2FuzzyLogicSystem::FuzzyRule rule;
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

double FuzzyGT2Adapter::compute(double y, double yref, double dt)
{
    if (!initialized_) {
        // Auto-configure with defaults if not explicitly configured
        configureDefault(true);
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

    // Evaluate GT2-FLS (alpha-plane method)
    double u = fls_.calculateOutput(error, derr, wind_val);

    // Handle NaN/Inf
    if (!std::isfinite(u)) {
        u = 0.0;
    }

    // Clamp to actuator limits
    u = std::clamp(u, opt_.umin, opt_.umax);

    return u;
}

void FuzzyGT2Adapter::reset()
{
    first_ = true;
    prev_error_ = 0.0;
}

void FuzzyGT2Adapter::setNumAlphaLevels(int n)
{
    opt_.num_alpha_levels = n;

    // Update config if already initialized
    if (initialized_) {
        GT2FuzzyLogicSystem::GT2Config config;
        config.num_alpha_levels = n;
        config.use_uniform_alpha = true;
        config.default_secondary_shape = opt_.secondary_shape;
        config.secondary_spread = opt_.secondary_spread;
        fls_.setConfig(config);
    }
}

void FuzzyGT2Adapter::setSecondaryShape(GT2FuzzyLogicSystem::SecondaryMFShape shape)
{
    opt_.secondary_shape = shape;
}

} // namespace agent_control_pkg::controllers
