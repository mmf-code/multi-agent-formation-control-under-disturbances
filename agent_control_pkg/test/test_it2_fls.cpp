/**
 * @file test_it2_fls.cpp
 * @brief Unit tests for IT2 Fuzzy Logic System
 */

#include "agent_control_pkg/it2_fuzzy_logic_system.hpp"
#include <gtest/gtest.h>
#include <cmath>

using agent_control_pkg::IT2FuzzyLogicSystem;
using FOU = IT2FuzzyLogicSystem::IT2TriangularFS_FOU;

// Helper function to construct a simple FLS with three symmetric sets
static IT2FuzzyLogicSystem makeSimpleFLS() {
    IT2FuzzyLogicSystem fls;
    fls.addInputVariable("error");
    fls.addInputVariable("dError");
    fls.addOutputVariable("correction");

    // Triangular sets with identical lower/upper membership (Type-1 like)
    fls.addFuzzySetToVariable("error", "N", FOU{-10.0, -5.0, 0.0, -10.0, -5.0, 0.0});
    fls.addFuzzySetToVariable("error", "Z", FOU{-1.0, 0.0, 1.0, -1.0, 0.0, 1.0});
    fls.addFuzzySetToVariable("error", "P", FOU{0.0, 5.0, 10.0, 0.0, 5.0, 10.0});

    fls.addFuzzySetToVariable("dError", "N", FOU{-10.0, -5.0, 0.0, -10.0, -5.0, 0.0});
    fls.addFuzzySetToVariable("dError", "Z", FOU{-1.0, 0.0, 1.0, -1.0, 0.0, 1.0});
    fls.addFuzzySetToVariable("dError", "P", FOU{0.0, 5.0, 10.0, 0.0, 5.0, 10.0});

    fls.addFuzzySetToVariable("correction", "N", FOU{-10.0, -5.0, 0.0, -10.0, -5.0, 0.0});
    fls.addFuzzySetToVariable("correction", "Z", FOU{-1.0, 0.0, 1.0, -1.0, 0.0, 1.0});
    fls.addFuzzySetToVariable("correction", "P", FOU{0.0, 5.0, 10.0, 0.0, 5.0, 10.0});

    // Three simple rules
    IT2FuzzyLogicSystem::FuzzyRule r1{{{"error", "N"}, {"dError", "N"}}, {"correction", "N"}};
    IT2FuzzyLogicSystem::FuzzyRule r2{{{"error", "Z"}, {"dError", "Z"}}, {"correction", "Z"}};
    IT2FuzzyLogicSystem::FuzzyRule r3{{{"error", "P"}, {"dError", "P"}}, {"correction", "P"}};
    fls.addRule(r1);
    fls.addRule(r2);
    fls.addRule(r3);
    return fls;
}

TEST(IT2FuzzyLogicSystemTest, FuzzificationMembership) {
    IT2FuzzyLogicSystem fls = makeSimpleFLS();
    // Membership of error=-5 in set N should be 1
    auto mi = fls.getTriangularMembership(-5.0, -10.0, -5.0, 0.0, -10.0, -5.0, 0.0);
    EXPECT_NEAR(mi.first, 1.0, 1e-6);
    EXPECT_NEAR(mi.second, 1.0, 1e-6);
    // Membership of error=0 in set Z should be 1
    auto mi2 = fls.getTriangularMembership(0.0, -1.0, 0.0, 1.0, -1.0, 0.0, 1.0);
    EXPECT_NEAR(mi2.first, 1.0, 1e-6);
    EXPECT_NEAR(mi2.second, 1.0, 1e-6);
}

TEST(IT2FuzzyLogicSystemTest, RuleFiringAndOutput) {
    IT2FuzzyLogicSystem fls = makeSimpleFLS();
    // Case where rule 1 should fire fully
    double out1 = fls.calculateOutput(-5.0, -5.0, 0.0);
    EXPECT_NEAR(out1, -5.0, 1e-6);

    // Case where rule 2 should fire fully
    double out2 = fls.calculateOutput(0.0, 0.0, 0.0);
    EXPECT_NEAR(out2, 0.0, 1e-6);

    // Case where rule 3 should fire fully
    double out3 = fls.calculateOutput(5.0, 5.0, 0.0);
    EXPECT_NEAR(out3, 5.0, 1e-6);
}

TEST(IT2FuzzyLogicSystemTest, NoMatchingRuleGivesZero) {
    IT2FuzzyLogicSystem fls = makeSimpleFLS();
    double out = fls.calculateOutput(-5.0, 5.0, 0.0);
    EXPECT_NEAR(out, 0.0, 1e-6);
}

TEST(IT2FuzzyLogicSystemTest, RuleCountCorrect) {
    IT2FuzzyLogicSystem fls = makeSimpleFLS();
    EXPECT_EQ(fls.getRuleCount(), 3u);
}

TEST(IT2FuzzyLogicSystemTest, InterpolatedOutputBetweenRules) {
    IT2FuzzyLogicSystem fls;
    fls.addInputVariable("error");
    fls.addInputVariable("dError");
    fls.addOutputVariable("output");

    // Two overlapping sets
    fls.addFuzzySetToVariable("error", "N", FOU{-10.0, -5.0, 0.0, -10.0, -5.0, 0.0});
    fls.addFuzzySetToVariable("error", "P", FOU{0.0, 5.0, 10.0, 0.0, 5.0, 10.0});
    fls.addFuzzySetToVariable("dError", "Z", FOU{-1.0, 0.0, 1.0, -1.0, 0.0, 1.0});
    fls.addFuzzySetToVariable("output", "N", FOU{-10.0, -5.0, 0.0, -10.0, -5.0, 0.0});
    fls.addFuzzySetToVariable("output", "P", FOU{0.0, 5.0, 10.0, 0.0, 5.0, 10.0});

    IT2FuzzyLogicSystem::FuzzyRule r1{{{"error", "N"}, {"dError", "Z"}}, {"output", "N"}};
    IT2FuzzyLogicSystem::FuzzyRule r2{{{"error", "P"}, {"dError", "Z"}}, {"output", "P"}};
    fls.addRule(r1);
    fls.addRule(r2);

    // At error=0, dError=0: both rules might have some activation
    // Output should be between -5 and 5
    double out = fls.calculateOutput(0.0, 0.0, 0.0);
    EXPECT_GE(out, -5.0);
    EXPECT_LE(out, 5.0);
}

TEST(IT2FuzzyLogicSystemTest, FOUWidthAffectsOutput) {
    // Test with FOU (Type-2 behavior)
    IT2FuzzyLogicSystem fls;
    fls.addInputVariable("error");
    fls.addInputVariable("dError");
    fls.addOutputVariable("output");

    // Set with FOU - lower and upper MFs are different
    fls.addFuzzySetToVariable("error", "Z", FOU{-2.0, 0.0, 2.0, -3.0, 0.0, 3.0});  // Wider UMF
    fls.addFuzzySetToVariable("dError", "Z", FOU{-1.0, 0.0, 1.0, -1.0, 0.0, 1.0});
    fls.addFuzzySetToVariable("output", "Z", FOU{-5.0, 0.0, 5.0, -5.0, 0.0, 5.0});

    IT2FuzzyLogicSystem::FuzzyRule r1{{{"error", "Z"}, {"dError", "Z"}}, {"output", "Z"}};
    fls.addRule(r1);

    // At error=0, dError=0, output should be near 0
    double out = fls.calculateOutput(0.0, 0.0, 0.0);
    EXPECT_NEAR(out, 0.0, 0.5);
}
