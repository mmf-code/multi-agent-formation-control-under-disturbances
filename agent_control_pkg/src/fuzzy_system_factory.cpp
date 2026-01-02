/**
 * @file fuzzy_system_factory.cpp
 * @brief Implementation of fuzzy system factory
 *
 * Creates IT2 and GT2 fuzzy systems with standard rule bases
 * for drone disturbance compensation.
 *
 * @author Multi-Agent Formation Control Research
 * @date 2026-01-03
 */

#include "agent_control_pkg/fuzzy_system_factory.hpp"
#include <iostream>

namespace agent_control_pkg {

// ========== Factory Main Method ==========

std::unique_ptr<FuzzySystemInterface> FuzzySystemFactory::create(
    FuzzySystemType type,
    int num_alpha_levels,
    GT2FuzzyLogicSystem::SecondaryMFShape secondary_shape)
{
    switch (type) {
        case FuzzySystemType::GT2:
            return createGT2(num_alpha_levels, secondary_shape);
        case FuzzySystemType::IT2:
        default:
            return createIT2();
    }
}

// ========== IT2 Creation ==========

std::unique_ptr<IT2FuzzyAdapter> FuzzySystemFactory::createIT2()
{
    auto system = std::make_shared<IT2FuzzyLogicSystem>();

    // Add variables
    system->addInputVariable("error");
    system->addInputVariable("dError");
    system->addInputVariable("wind");
    system->addOutputVariable("output");

    // Configure standard sets
    configureIT2Sets(system.get());

    // Add rules
    addStandardRules(system.get());

    return std::make_unique<IT2FuzzyAdapter>(system);
}

// ========== GT2 Creation ==========

std::unique_ptr<GT2FuzzyAdapter> FuzzySystemFactory::createGT2(
    int num_alpha_levels,
    GT2FuzzyLogicSystem::SecondaryMFShape secondary_shape)
{
    GT2FuzzyLogicSystem::GT2Config config;
    config.num_alpha_levels = num_alpha_levels;
    config.use_uniform_alpha = true;
    config.default_secondary_shape = secondary_shape;
    config.secondary_spread = 0.3;  // For Gaussian

    auto system = std::make_shared<GT2FuzzyLogicSystem>(config);

    // Add variables
    system->addInputVariable("error");
    system->addInputVariable("dError");
    system->addInputVariable("wind");
    system->addOutputVariable("output");

    // Configure standard sets
    configureGT2Sets(system.get(), secondary_shape);

    // Add rules
    addStandardRules(system.get());

    return std::make_unique<GT2FuzzyAdapter>(system);
}

// ========== IT2 Sets Configuration ==========

void FuzzySystemFactory::configureIT2Sets(IT2FuzzyLogicSystem* system)
{
    using FOU = IT2FuzzyLogicSystem::IT2TriangularFS_FOU;

    // Error sets (normalized to [-1, 1] range)
    // NB: Negative Big, NS: Negative Small, ZO: Zero, PS: Positive Small, PB: Positive Big
    system->addFuzzySetToVariable("error", "NB", FOU{-1.2, -1.0, -0.4, -1.0, -0.8, -0.2});
    system->addFuzzySetToVariable("error", "NS", FOU{-0.6, -0.4, 0.0, -0.5, -0.3, 0.1});
    system->addFuzzySetToVariable("error", "ZO", FOU{-0.2, 0.0, 0.2, -0.15, 0.0, 0.15});
    system->addFuzzySetToVariable("error", "PS", FOU{0.0, 0.4, 0.6, -0.1, 0.3, 0.5});
    system->addFuzzySetToVariable("error", "PB", FOU{0.4, 1.0, 1.2, 0.2, 0.8, 1.0});

    // dError sets (error derivative)
    system->addFuzzySetToVariable("dError", "NB", FOU{-1.2, -1.0, -0.4, -1.0, -0.8, -0.2});
    system->addFuzzySetToVariable("dError", "NS", FOU{-0.6, -0.4, 0.0, -0.5, -0.3, 0.1});
    system->addFuzzySetToVariable("dError", "ZO", FOU{-0.2, 0.0, 0.2, -0.15, 0.0, 0.15});
    system->addFuzzySetToVariable("dError", "PS", FOU{0.0, 0.4, 0.6, -0.1, 0.3, 0.5});
    system->addFuzzySetToVariable("dError", "PB", FOU{0.4, 1.0, 1.2, 0.2, 0.8, 1.0});

    // Wind sets (disturbance level)
    system->addFuzzySetToVariable("wind", "LOW",    FOU{0.0, 0.0, 0.3, 0.0, 0.0, 0.4});
    system->addFuzzySetToVariable("wind", "MEDIUM", FOU{0.2, 0.5, 0.8, 0.1, 0.5, 0.9});
    system->addFuzzySetToVariable("wind", "HIGH",   FOU{0.7, 1.0, 1.2, 0.6, 1.0, 1.0});

    // Output sets (control adjustment)
    system->addFuzzySetToVariable("output", "NB", FOU{-1.2, -1.0, -0.6, -1.0, -0.9, -0.5});
    system->addFuzzySetToVariable("output", "NS", FOU{-0.7, -0.4, -0.1, -0.6, -0.35, -0.05});
    system->addFuzzySetToVariable("output", "ZO", FOU{-0.15, 0.0, 0.15, -0.1, 0.0, 0.1});
    system->addFuzzySetToVariable("output", "PS", FOU{0.1, 0.4, 0.7, 0.05, 0.35, 0.6});
    system->addFuzzySetToVariable("output", "PB", FOU{0.6, 1.0, 1.2, 0.5, 0.9, 1.0});
}

// ========== GT2 Sets Configuration ==========

void FuzzySystemFactory::configureGT2Sets(
    GT2FuzzyLogicSystem* system,
    GT2FuzzyLogicSystem::SecondaryMFShape shape)
{
    using FS = GT2FuzzyLogicSystem::GT2TriangularFS;

    // Helper to create GT2 set with specified secondary shape
    auto makeSet = [shape](double ll, double lp, double lr,
                           double ul, double up, double ur) -> FS {
        FS fs;
        fs.lmf_left_base = ll;
        fs.lmf_peak = lp;
        fs.lmf_right_base = lr;
        fs.umf_left_base = ul;
        fs.umf_peak = up;
        fs.umf_right_base = ur;
        fs.secondary_shape = shape;
        fs.secondary_spread = 0.3;  // For Gaussian
        fs.secondary_peak_location = 0.5;  // Center of FOU
        return fs;
    };

    // Error sets
    system->addFuzzySetToVariable("error", "NB", makeSet(-1.2, -1.0, -0.4, -1.0, -0.8, -0.2));
    system->addFuzzySetToVariable("error", "NS", makeSet(-0.6, -0.4, 0.0, -0.5, -0.3, 0.1));
    system->addFuzzySetToVariable("error", "ZO", makeSet(-0.2, 0.0, 0.2, -0.15, 0.0, 0.15));
    system->addFuzzySetToVariable("error", "PS", makeSet(0.0, 0.4, 0.6, -0.1, 0.3, 0.5));
    system->addFuzzySetToVariable("error", "PB", makeSet(0.4, 1.0, 1.2, 0.2, 0.8, 1.0));

    // dError sets
    system->addFuzzySetToVariable("dError", "NB", makeSet(-1.2, -1.0, -0.4, -1.0, -0.8, -0.2));
    system->addFuzzySetToVariable("dError", "NS", makeSet(-0.6, -0.4, 0.0, -0.5, -0.3, 0.1));
    system->addFuzzySetToVariable("dError", "ZO", makeSet(-0.2, 0.0, 0.2, -0.15, 0.0, 0.15));
    system->addFuzzySetToVariable("dError", "PS", makeSet(0.0, 0.4, 0.6, -0.1, 0.3, 0.5));
    system->addFuzzySetToVariable("dError", "PB", makeSet(0.4, 1.0, 1.2, 0.2, 0.8, 1.0));

    // Wind sets
    system->addFuzzySetToVariable("wind", "LOW",    makeSet(0.0, 0.0, 0.3, 0.0, 0.0, 0.4));
    system->addFuzzySetToVariable("wind", "MEDIUM", makeSet(0.2, 0.5, 0.8, 0.1, 0.5, 0.9));
    system->addFuzzySetToVariable("wind", "HIGH",   makeSet(0.7, 1.0, 1.2, 0.6, 1.0, 1.0));

    // Output sets
    system->addFuzzySetToVariable("output", "NB", makeSet(-1.2, -1.0, -0.6, -1.0, -0.9, -0.5));
    system->addFuzzySetToVariable("output", "NS", makeSet(-0.7, -0.4, -0.1, -0.6, -0.35, -0.05));
    system->addFuzzySetToVariable("output", "ZO", makeSet(-0.15, 0.0, 0.15, -0.1, 0.0, 0.1));
    system->addFuzzySetToVariable("output", "PS", makeSet(0.1, 0.4, 0.7, 0.05, 0.35, 0.6));
    system->addFuzzySetToVariable("output", "PB", makeSet(0.6, 1.0, 1.2, 0.5, 0.9, 1.0));
}

// ========== Standard Rule Base ==========

template<typename SystemType>
void FuzzySystemFactory::addStandardRules(SystemType* system)
{
    using Rule = typename SystemType::FuzzyRule;

    // 5x5 rule matrix for error × dError (under low wind)
    // Output is acceleration adjustment
    //
    //          dError
    //        NB   NS   ZO   PS   PB
    // e NB   NB   NB   NS   NS   ZO
    // r NS   NB   NS   NS   ZO   PS
    // r ZO   NS   NS   ZO   PS   PS
    // o PS   NS   ZO   PS   PS   PB
    // r PB   ZO   PS   PS   PB   PB

    // Standard 25 rules (low wind)
    std::vector<std::tuple<std::string, std::string, std::string, std::string>> rules_low = {
        // (error, dError, wind, output)
        {"NB", "NB", "LOW", "NB"}, {"NB", "NS", "LOW", "NB"}, {"NB", "ZO", "LOW", "NS"},
        {"NB", "PS", "LOW", "NS"}, {"NB", "PB", "LOW", "ZO"},

        {"NS", "NB", "LOW", "NB"}, {"NS", "NS", "LOW", "NS"}, {"NS", "ZO", "LOW", "NS"},
        {"NS", "PS", "LOW", "ZO"}, {"NS", "PB", "LOW", "PS"},

        {"ZO", "NB", "LOW", "NS"}, {"ZO", "NS", "LOW", "NS"}, {"ZO", "ZO", "LOW", "ZO"},
        {"ZO", "PS", "LOW", "PS"}, {"ZO", "PB", "LOW", "PS"},

        {"PS", "NB", "LOW", "NS"}, {"PS", "NS", "LOW", "ZO"}, {"PS", "ZO", "LOW", "PS"},
        {"PS", "PS", "LOW", "PS"}, {"PS", "PB", "LOW", "PB"},

        {"PB", "NB", "LOW", "ZO"}, {"PB", "NS", "LOW", "PS"}, {"PB", "ZO", "LOW", "PS"},
        {"PB", "PS", "LOW", "PB"}, {"PB", "PB", "LOW", "PB"},
    };

    // Medium wind - more aggressive response
    std::vector<std::tuple<std::string, std::string, std::string, std::string>> rules_medium = {
        {"NB", "NB", "MEDIUM", "NB"}, {"NB", "NS", "MEDIUM", "NB"}, {"NB", "ZO", "MEDIUM", "NB"},
        {"NB", "PS", "MEDIUM", "NS"}, {"NB", "PB", "MEDIUM", "NS"},

        {"NS", "NB", "MEDIUM", "NB"}, {"NS", "NS", "MEDIUM", "NB"}, {"NS", "ZO", "MEDIUM", "NS"},
        {"NS", "PS", "MEDIUM", "NS"}, {"NS", "PB", "MEDIUM", "ZO"},

        {"ZO", "NB", "MEDIUM", "NS"}, {"ZO", "NS", "MEDIUM", "NS"}, {"ZO", "ZO", "MEDIUM", "ZO"},
        {"ZO", "PS", "MEDIUM", "PS"}, {"ZO", "PB", "MEDIUM", "PS"},

        {"PS", "NB", "MEDIUM", "ZO"}, {"PS", "NS", "MEDIUM", "PS"}, {"PS", "ZO", "MEDIUM", "PS"},
        {"PS", "PS", "MEDIUM", "PB"}, {"PS", "PB", "MEDIUM", "PB"},

        {"PB", "NB", "MEDIUM", "NS"}, {"PB", "NS", "MEDIUM", "PS"}, {"PB", "ZO", "MEDIUM", "PB"},
        {"PB", "PS", "MEDIUM", "PB"}, {"PB", "PB", "MEDIUM", "PB"},
    };

    // High wind - maximum response
    std::vector<std::tuple<std::string, std::string, std::string, std::string>> rules_high = {
        {"NB", "NB", "HIGH", "NB"}, {"NB", "NS", "HIGH", "NB"}, {"NB", "ZO", "HIGH", "NB"},
        {"NB", "PS", "HIGH", "NB"}, {"NB", "PB", "HIGH", "NS"},

        {"NS", "NB", "HIGH", "NB"}, {"NS", "NS", "HIGH", "NB"}, {"NS", "ZO", "HIGH", "NB"},
        {"NS", "PS", "HIGH", "NS"}, {"NS", "PB", "HIGH", "ZO"},

        {"ZO", "NB", "HIGH", "NB"}, {"ZO", "NS", "HIGH", "NS"}, {"ZO", "ZO", "HIGH", "ZO"},
        {"ZO", "PS", "HIGH", "PS"}, {"ZO", "PB", "HIGH", "PB"},

        {"PS", "NB", "HIGH", "ZO"}, {"PS", "NS", "HIGH", "PS"}, {"PS", "ZO", "HIGH", "PB"},
        {"PS", "PS", "HIGH", "PB"}, {"PS", "PB", "HIGH", "PB"},

        {"PB", "NB", "HIGH", "NS"}, {"PB", "NS", "HIGH", "PB"}, {"PB", "ZO", "HIGH", "PB"},
        {"PB", "PS", "HIGH", "PB"}, {"PB", "PB", "HIGH", "PB"},
    };

    // Add all rules
    auto addRules = [system](const auto& rule_list) {
        for (const auto& [err, derr, wind, out] : rule_list) {
            Rule rule;
            rule.antecedents["error"] = err;
            rule.antecedents["dError"] = derr;
            rule.antecedents["wind"] = wind;
            rule.consequent = {"output", out};
            system->addRule(rule);
        }
    };

    addRules(rules_low);
    addRules(rules_medium);
    addRules(rules_high);
}

// Explicit template instantiations
template void FuzzySystemFactory::addStandardRules<IT2FuzzyLogicSystem>(IT2FuzzyLogicSystem*);
template void FuzzySystemFactory::addStandardRules<GT2FuzzyLogicSystem>(GT2FuzzyLogicSystem*);

} // namespace agent_control_pkg
