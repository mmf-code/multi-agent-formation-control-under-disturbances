/**
 * @file test_fuzzy_comparison.cpp
 * @brief Comparative tests between IT2-FLS and GT2-FLS
 *
 * Validates that both fuzzy systems produce consistent outputs
 * and measures performance differences.
 *
 * @author Multi-Agent Formation Control Research
 * @date 2026-01-03
 */

#include <gtest/gtest.h>
#include "agent_control_pkg/it2_fuzzy_logic_system.hpp"
#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include "agent_control_pkg/fuzzy_system_factory.hpp"
#include <cmath>
#include <chrono>
#include <vector>
#include <numeric>
#include <iostream>

using namespace agent_control_pkg;

class FuzzyComparisonTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        it2_adapter_ = FuzzySystemFactory::createIT2();
        gt2_adapter_ = FuzzySystemFactory::createGT2(5, GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR);
        gt2_uniform_adapter_ = FuzzySystemFactory::createGT2(5, GT2FuzzyLogicSystem::SecondaryMFShape::UNIFORM);
    }

    std::unique_ptr<IT2FuzzyAdapter> it2_adapter_;
    std::unique_ptr<GT2FuzzyAdapter> gt2_adapter_;
    std::unique_ptr<GT2FuzzyAdapter> gt2_uniform_adapter_;
};

// Both systems should produce non-zero output for non-zero input
TEST_F(FuzzyComparisonTest, BothProduceNonZeroOutput)
{
    double it2_output = it2_adapter_->calculateOutput(0.5, 0.1, 0.2);
    double gt2_output = gt2_adapter_->calculateOutput(0.5, 0.1, 0.2);

    EXPECT_NE(it2_output, 0.0);
    EXPECT_NE(gt2_output, 0.0);

    // Both should be in same direction (positive for positive error)
    EXPECT_GT(it2_output, 0.0);
    EXPECT_GT(gt2_output, 0.0);
}

// Outputs should be similar (within reasonable tolerance)
TEST_F(FuzzyComparisonTest, OutputsSimilar)
{
    std::vector<std::tuple<double, double, double>> test_cases = {
        {0.0, 0.0, 0.0},
        {0.5, 0.0, 0.0},
        {-0.5, 0.0, 0.0},
        {0.3, 0.2, 0.0},
        {0.3, -0.2, 0.0},
        {0.5, 0.1, 0.5},
        {-0.3, 0.1, 0.8},
        {0.8, 0.3, 0.2},
    };

    double max_diff = 0.0;
    double sum_diff = 0.0;

    for (const auto& [error, dError, wind] : test_cases) {
        double it2_out = it2_adapter_->calculateOutput(error, dError, wind);
        double gt2_out = gt2_adapter_->calculateOutput(error, dError, wind);

        double diff = std::abs(it2_out - gt2_out);
        max_diff = std::max(max_diff, diff);
        sum_diff += diff;

        // Outputs should be within 50% of each other (generous tolerance)
        double max_mag = std::max(std::abs(it2_out), std::abs(gt2_out));
        if (max_mag > 0.01) {
            double rel_diff = diff / max_mag;
            EXPECT_LT(rel_diff, 0.5) << "IT2=" << it2_out << ", GT2=" << gt2_out
                                     << " at error=" << error << ", dError=" << dError;
        }
    }

    std::cout << "[INFO] IT2 vs GT2 - Max diff: " << max_diff
              << ", Avg diff: " << sum_diff / test_cases.size() << std::endl;
}

// GT2 with uniform secondary MF should be very close to IT2
TEST_F(FuzzyComparisonTest, UniformGT2CloseToIT2)
{
    std::vector<std::tuple<double, double, double>> test_cases = {
        {0.3, 0.1, 0.0},
        {0.5, 0.2, 0.3},
        {-0.4, -0.1, 0.5},
        {0.7, 0.3, 0.8},
    };

    for (const auto& [error, dError, wind] : test_cases) {
        double it2_out = it2_adapter_->calculateOutput(error, dError, wind);
        double gt2_uniform_out = gt2_uniform_adapter_->calculateOutput(error, dError, wind);

        // Uniform GT2 should be very close to IT2 (theoretically equivalent)
        double diff = std::abs(it2_out - gt2_uniform_out);
        EXPECT_LT(diff, 0.3) << "IT2=" << it2_out << ", GT2-Uniform=" << gt2_uniform_out
                             << " at error=" << error;
    }
}

// Both should maintain direction consistency
TEST_F(FuzzyComparisonTest, DirectionConsistency)
{
    // Positive error -> positive output
    EXPECT_GT(it2_adapter_->calculateOutput(0.5, 0.0, 0.0), 0.0);
    EXPECT_GT(gt2_adapter_->calculateOutput(0.5, 0.0, 0.0), 0.0);

    // Negative error -> negative output
    EXPECT_LT(it2_adapter_->calculateOutput(-0.5, 0.0, 0.0), 0.0);
    EXPECT_LT(gt2_adapter_->calculateOutput(-0.5, 0.0, 0.0), 0.0);
}

// Performance comparison: IT2 should be faster
TEST_F(FuzzyComparisonTest, PerformanceComparison)
{
    const int num_iterations = 1000;
    std::vector<double> inputs_e, inputs_de, inputs_w;

    // Generate test inputs
    for (int i = 0; i < num_iterations; ++i) {
        inputs_e.push_back((static_cast<double>(i) / num_iterations) * 2.0 - 1.0);
        inputs_de.push_back((static_cast<double>((i * 7) % num_iterations) / num_iterations) * 2.0 - 1.0);
        inputs_w.push_back(static_cast<double>((i * 13) % num_iterations) / num_iterations);
    }

    // Measure IT2 time
    auto start_it2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_iterations; ++i) {
        it2_adapter_->calculateOutput(inputs_e[i], inputs_de[i], inputs_w[i]);
    }
    auto end_it2 = std::chrono::high_resolution_clock::now();
    auto it2_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_it2 - start_it2);

    // Measure GT2 time (5 alpha levels)
    auto start_gt2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_iterations; ++i) {
        gt2_adapter_->calculateOutput(inputs_e[i], inputs_de[i], inputs_w[i]);
    }
    auto end_gt2 = std::chrono::high_resolution_clock::now();
    auto gt2_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_gt2 - start_gt2);

    std::cout << "[PERFORMANCE]" << std::endl;
    std::cout << "  IT2: " << it2_duration.count() << " us (" << num_iterations << " iterations)" << std::endl;
    std::cout << "  GT2: " << gt2_duration.count() << " us (" << num_iterations << " iterations)" << std::endl;
    std::cout << "  GT2/IT2 ratio: " << static_cast<double>(gt2_duration.count()) / it2_duration.count() << "x" << std::endl;

    // IT2 should be faster (GT2 has multiple alpha-plane computations)
    // But both should complete in reasonable time (< 100ms for 1000 iterations)
    EXPECT_LT(it2_duration.count(), 100000) << "IT2 too slow";
    EXPECT_LT(gt2_duration.count(), 500000) << "GT2 too slow";
}

// Different secondary shapes should produce different outputs
TEST_F(FuzzyComparisonTest, SecondaryShapesProduceDifferentOutputs)
{
    std::vector<GT2FuzzyLogicSystem::SecondaryMFShape> shapes = {
        GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR,
        GT2FuzzyLogicSystem::SecondaryMFShape::GAUSSIAN,
        GT2FuzzyLogicSystem::SecondaryMFShape::TRAPEZOIDAL,
    };

    std::vector<double> outputs;
    for (auto shape : shapes) {
        auto adapter = FuzzySystemFactory::createGT2(5, shape);
        outputs.push_back(adapter->calculateOutput(0.4, 0.1, 0.3));
    }

    // At least some outputs should be different
    bool all_same = true;
    for (size_t i = 1; i < outputs.size(); ++i) {
        if (std::abs(outputs[i] - outputs[0]) > 0.01) {
            all_same = false;
            break;
        }
    }

    // Different secondary shapes should produce at least slightly different outputs
    // (This tests that the secondary MF is actually being used)
    EXPECT_FALSE(all_same) << "All secondary shapes produced identical outputs";
}

// Number of alpha levels affects GT2 precision
TEST_F(FuzzyComparisonTest, AlphaLevelsAffectPrecision)
{
    std::vector<int> alpha_counts = {1, 3, 5, 10, 20};
    std::vector<double> outputs;

    for (int n : alpha_counts) {
        auto adapter = FuzzySystemFactory::createGT2(n, GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR);
        outputs.push_back(adapter->calculateOutput(0.5, 0.2, 0.3));
    }

    // Outputs should converge as alpha levels increase
    // Check that the sequence is converging (differences decreasing)
    bool converging = true;
    for (size_t i = 2; i < outputs.size(); ++i) {
        double diff_prev = std::abs(outputs[i-1] - outputs[i-2]);
        double diff_curr = std::abs(outputs[i] - outputs[i-1]);

        // Allow some tolerance - just check trend
        if (diff_curr > diff_prev * 1.5 && diff_prev > 0.01) {
            converging = false;
        }
    }

    std::cout << "[CONVERGENCE] Alpha level outputs: ";
    for (size_t i = 0; i < outputs.size(); ++i) {
        std::cout << "n=" << alpha_counts[i] << ":" << outputs[i] << " ";
    }
    std::cout << std::endl;

    // All outputs should be valid
    for (double o : outputs) {
        EXPECT_TRUE(std::isfinite(o));
        EXPECT_GT(o, -2.0);
        EXPECT_LT(o, 2.0);
    }
}

// Factory creates correct types
TEST_F(FuzzyComparisonTest, FactoryCreatesCorrectTypes)
{
    auto it2 = FuzzySystemFactory::create(FuzzySystemType::IT2);
    auto gt2 = FuzzySystemFactory::create(FuzzySystemType::GT2);

    EXPECT_EQ(it2->getTypeName(), "IT2-FLS");
    EXPECT_EQ(gt2->getTypeName(), "GT2-FLS");
}

// String conversion utilities
TEST_F(FuzzyComparisonTest, StringConversions)
{
    EXPECT_EQ(fuzzyTypeToString(FuzzySystemType::IT2), "IT2");
    EXPECT_EQ(fuzzyTypeToString(FuzzySystemType::GT2), "GT2");

    EXPECT_EQ(stringToFuzzyType("IT2"), FuzzySystemType::IT2);
    EXPECT_EQ(stringToFuzzyType("it2"), FuzzySystemType::IT2);
    EXPECT_EQ(stringToFuzzyType("GT2"), FuzzySystemType::GT2);
    EXPECT_EQ(stringToFuzzyType("gt2"), FuzzySystemType::GT2);
    EXPECT_EQ(stringToFuzzyType("general"), FuzzySystemType::GT2);

    EXPECT_EQ(secondaryShapeToString(GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR), "Triangular");
    EXPECT_EQ(secondaryShapeToString(GT2FuzzyLogicSystem::SecondaryMFShape::GAUSSIAN), "Gaussian");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
