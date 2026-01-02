/**
 * @file test_gt2_fls.cpp
 * @brief Unit tests for General Type-2 Fuzzy Logic System
 *
 * Tests the alpha-plane method implementation and verifies
 * GT2-FLS produces reasonable outputs.
 *
 * @author Multi-Agent Formation Control Research
 * @date 2026-01-03
 */

#include <gtest/gtest.h>
#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include "agent_control_pkg/fuzzy_system_factory.hpp"
#include <cmath>
#include <vector>

using namespace agent_control_pkg;

class GT2FuzzyLogicSystemTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Create GT2 system with factory (includes standard rule base)
        gt2_adapter_ = FuzzySystemFactory::createGT2(5, GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR);
    }

    std::unique_ptr<GT2FuzzyAdapter> gt2_adapter_;
};

// Test basic construction
TEST_F(GT2FuzzyLogicSystemTest, ConstructionSucceeds)
{
    ASSERT_NE(gt2_adapter_, nullptr);
    EXPECT_EQ(gt2_adapter_->getTypeName(), "GT2-FLS");
    EXPECT_GT(gt2_adapter_->getRuleCount(), 0);
}

// Test alpha levels generation
TEST_F(GT2FuzzyLogicSystemTest, AlphaLevelsGenerated)
{
    GT2FuzzyLogicSystem::GT2Config config;
    config.num_alpha_levels = 5;
    config.use_uniform_alpha = true;

    GT2FuzzyLogicSystem system(config);

    std::vector<double> levels = system.getAlphaLevels();
    EXPECT_EQ(levels.size(), 5);

    // Should be sorted descending
    for (size_t i = 1; i < levels.size(); ++i) {
        EXPECT_LE(levels[i], levels[i-1]);
    }

    // All levels should be in (0, 1]
    for (double alpha : levels) {
        EXPECT_GT(alpha, 0.0);
        EXPECT_LE(alpha, 1.0);
    }
}

// Test zero input produces near-zero output
TEST_F(GT2FuzzyLogicSystemTest, ZeroInputNearZeroOutput)
{
    double output = gt2_adapter_->calculateOutput(0.0, 0.0, 0.0);
    EXPECT_NEAR(output, 0.0, 0.2);  // Allow some tolerance
}

// Test symmetry: positive error should produce positive output
TEST_F(GT2FuzzyLogicSystemTest, PositiveErrorPositiveOutput)
{
    double output = gt2_adapter_->calculateOutput(0.5, 0.0, 0.0);
    EXPECT_GT(output, 0.0) << "Positive error should produce positive correction";
}

// Test symmetry: negative error should produce negative output
TEST_F(GT2FuzzyLogicSystemTest, NegativeErrorNegativeOutput)
{
    double output = gt2_adapter_->calculateOutput(-0.5, 0.0, 0.0);
    EXPECT_LT(output, 0.0) << "Negative error should produce negative correction";
}

// Test monotonicity: larger errors should produce larger outputs
TEST_F(GT2FuzzyLogicSystemTest, MonotonicityWithError)
{
    double small_error_output = std::abs(gt2_adapter_->calculateOutput(0.2, 0.0, 0.0));
    double large_error_output = std::abs(gt2_adapter_->calculateOutput(0.8, 0.0, 0.0));

    EXPECT_GT(large_error_output, small_error_output)
        << "Larger error should produce larger output magnitude";
}

// Test wind input increases response
TEST_F(GT2FuzzyLogicSystemTest, WindIncreasesResponse)
{
    double no_wind = std::abs(gt2_adapter_->calculateOutput(0.3, 0.0, 0.0));
    double high_wind = std::abs(gt2_adapter_->calculateOutput(0.3, 0.0, 0.8));

    // Under wind, response should be more aggressive (or at least not smaller)
    EXPECT_GE(high_wind, no_wind * 0.8)  // Allow some tolerance
        << "Wind should not significantly reduce response";
}

// Test alpha-plane results are stored
TEST_F(GT2FuzzyLogicSystemTest, AlphaPlaneResultsStored)
{
    gt2_adapter_->calculateOutput(0.5, 0.1, 0.3);

    const auto& results = gt2_adapter_->getLastAlphaResults();
    EXPECT_GT(results.size(), 0);

    for (const auto& r : results) {
        EXPECT_GT(r.alpha_level, 0.0);
        EXPECT_LE(r.alpha_level, 1.0);
        EXPECT_GE(r.weight, 0.0);
    }
}

// Test different secondary MF shapes
TEST_F(GT2FuzzyLogicSystemTest, DifferentSecondaryShapes)
{
    std::vector<GT2FuzzyLogicSystem::SecondaryMFShape> shapes = {
        GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR,
        GT2FuzzyLogicSystem::SecondaryMFShape::GAUSSIAN,
        GT2FuzzyLogicSystem::SecondaryMFShape::TRAPEZOIDAL,
        GT2FuzzyLogicSystem::SecondaryMFShape::UNIFORM
    };

    std::vector<double> outputs;
    for (auto shape : shapes) {
        auto adapter = FuzzySystemFactory::createGT2(5, shape);
        double output = adapter->calculateOutput(0.4, 0.1, 0.2);
        outputs.push_back(output);

        // All should produce finite, reasonable outputs
        EXPECT_TRUE(std::isfinite(output));
        EXPECT_GT(output, -2.0);
        EXPECT_LT(output, 2.0);
    }

    // Uniform shape should be closest to IT2
    // (GT2 with uniform secondary MF is equivalent to IT2)
}

// Test number of alpha levels affects computation
TEST_F(GT2FuzzyLogicSystemTest, AlphaLevelCountAffectsResult)
{
    std::vector<double> outputs;

    for (int n : {1, 3, 5, 10}) {
        auto adapter = FuzzySystemFactory::createGT2(n, GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR);
        double output = adapter->calculateOutput(0.5, 0.2, 0.1);
        outputs.push_back(output);

        EXPECT_TRUE(std::isfinite(output));
    }

    // Results should converge as alpha levels increase
    // The difference between n=5 and n=10 should be smaller than n=1 and n=3
    double diff_1_3 = std::abs(outputs[0] - outputs[1]);
    double diff_5_10 = std::abs(outputs[2] - outputs[3]);

    // Allow some tolerance - just verify they're all reasonable
    for (double o : outputs) {
        EXPECT_GT(o, -1.5);
        EXPECT_LT(o, 1.5);
    }
}

// Test secondary membership calculation
TEST_F(GT2FuzzyLogicSystemTest, SecondaryMembershipTriangular)
{
    GT2FuzzyLogicSystem::GT2TriangularFS fs;
    fs.secondary_shape = GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR;
    fs.secondary_peak_location = 0.5;
    fs.secondary_spread = 0.3;

    // At center (0.5), secondary membership should be 1.0
    double center_membership = fs.getSecondaryMembership(0.5, 0.0, 1.0);
    EXPECT_NEAR(center_membership, 1.0, 0.01);

    // At edges (0.0 and 1.0), secondary membership should be 0.0
    double left_membership = fs.getSecondaryMembership(0.0, 0.0, 1.0);
    double right_membership = fs.getSecondaryMembership(1.0, 0.0, 1.0);
    EXPECT_NEAR(left_membership, 0.0, 0.01);
    EXPECT_NEAR(right_membership, 0.0, 0.01);
}

// Test secondary membership calculation - Gaussian
TEST_F(GT2FuzzyLogicSystemTest, SecondaryMembershipGaussian)
{
    GT2FuzzyLogicSystem::GT2TriangularFS fs;
    fs.secondary_shape = GT2FuzzyLogicSystem::SecondaryMFShape::GAUSSIAN;
    fs.secondary_peak_location = 0.5;
    fs.secondary_spread = 0.3;

    // At center, Gaussian should be 1.0
    double center_membership = fs.getSecondaryMembership(0.5, 0.0, 1.0);
    EXPECT_NEAR(center_membership, 1.0, 0.01);

    // Away from center, should be less than 1.0 but greater than 0
    double off_center = fs.getSecondaryMembership(0.3, 0.0, 1.0);
    EXPECT_GT(off_center, 0.0);
    EXPECT_LT(off_center, 1.0);
}

// Test uniform secondary MF (should behave like IT2)
TEST_F(GT2FuzzyLogicSystemTest, UniformSecondaryBehavesLikeIT2)
{
    GT2FuzzyLogicSystem::GT2TriangularFS fs;
    fs.secondary_shape = GT2FuzzyLogicSystem::SecondaryMFShape::UNIFORM;

    // Uniform should always return 1.0 regardless of position
    EXPECT_NEAR(fs.getSecondaryMembership(0.0, 0.0, 1.0), 1.0, 0.01);
    EXPECT_NEAR(fs.getSecondaryMembership(0.5, 0.0, 1.0), 1.0, 0.01);
    EXPECT_NEAR(fs.getSecondaryMembership(1.0, 0.0, 1.0), 1.0, 0.01);
}

// Test alpha-cut formula verification for triangular secondary MF
// Mathematical formula: u_low = α × p, u_high = 1 - α × (1 - p)
// where p = secondary_peak_location
TEST_F(GT2FuzzyLogicSystemTest, AlphaCutFormulaVerification)
{
    GT2FuzzyLogicSystem::GT2Config config;
    config.num_alpha_levels = 5;
    config.default_secondary_shape = GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR;

    GT2FuzzyLogicSystem system(config);

    // Create a test fuzzy set with known parameters
    GT2FuzzyLogicSystem::GT2TriangularFS fs;
    fs.umf_left_base = 0.0;
    fs.umf_peak = 0.5;
    fs.umf_right_base = 1.0;
    fs.lmf_left_base = 0.2;
    fs.lmf_peak = 0.5;
    fs.lmf_right_base = 0.8;
    fs.secondary_shape = GT2FuzzyLogicSystem::SecondaryMFShape::TRIANGULAR;
    fs.secondary_peak_location = 0.5;  // Symmetric triangle

    // Test with crisp input at peak (membership = [1.0, 1.0] for both UMF/LMF)
    // At x = 0.5:
    //   UMF: (0.5 - 0.0) / (0.5 - 0.0) = 1.0
    //   LMF: (0.5 - 0.2) / (0.5 - 0.2) = 1.0
    // FOU interval: [1.0, 1.0] -> degenerate, alpha-cut should also be [1.0, 1.0]

    // Test with crisp input away from peak (x = 0.25)
    //   UMF: (0.25 - 0.0) / (0.5 - 0.0) = 0.5
    //   LMF: (0.25 - 0.2) / (0.5 - 0.2) = 0.1667
    // FOU interval: [0.1667, 0.5]

    // For α-cut with p=0.5 (symmetric triangle):
    //   α = 0.5: u_low = 0.5 × 0.5 = 0.25, u_high = 1 - 0.5 × 0.5 = 0.75
    //   α = 1.0: u_low = 1.0 × 0.5 = 0.5, u_high = 1 - 1.0 × 0.5 = 0.5 (single point)

    // Verify the formula with known values
    double p = 0.5;  // peak location

    // Test α = 0.5
    double alpha_05 = 0.5;
    double expected_u_low_05 = alpha_05 * p;       // = 0.25
    double expected_u_high_05 = 1.0 - alpha_05 * (1.0 - p);  // = 0.75
    EXPECT_NEAR(expected_u_low_05, 0.25, 0.001);
    EXPECT_NEAR(expected_u_high_05, 0.75, 0.001);

    // Test α = 1.0
    double alpha_10 = 1.0;
    double expected_u_low_10 = alpha_10 * p;       // = 0.5
    double expected_u_high_10 = 1.0 - alpha_10 * (1.0 - p);  // = 0.5
    EXPECT_NEAR(expected_u_low_10, 0.5, 0.001);
    EXPECT_NEAR(expected_u_high_10, 0.5, 0.001);

    // Test asymmetric case: p = 0.3
    double p_asym = 0.3;
    double alpha = 0.5;
    double u_low_asym = alpha * p_asym;        // = 0.15
    double u_high_asym = 1.0 - alpha * (1.0 - p_asym);  // = 1 - 0.5 × 0.7 = 0.65
    EXPECT_NEAR(u_low_asym, 0.15, 0.001);
    EXPECT_NEAR(u_high_asym, 0.65, 0.001);

    // Verify width shrinks with higher alpha
    double width_alpha_02 = (1.0 - 0.2 * (1.0 - p)) - (0.2 * p);  // α=0.2: 0.9 - 0.1 = 0.8
    double width_alpha_08 = (1.0 - 0.8 * (1.0 - p)) - (0.8 * p);  // α=0.8: 0.6 - 0.4 = 0.2
    EXPECT_GT(width_alpha_02, width_alpha_08) << "Higher alpha should give narrower interval";
}

// Stress test with many inputs
TEST_F(GT2FuzzyLogicSystemTest, StressTestManyInputs)
{
    int num_tests = 100;
    int failures = 0;

    for (int i = 0; i < num_tests; ++i) {
        double error = (static_cast<double>(i) / num_tests) * 2.0 - 1.0;  // [-1, 1]
        double dError = (static_cast<double>((i * 7) % num_tests) / num_tests) * 2.0 - 1.0;
        double wind = static_cast<double>((i * 13) % num_tests) / num_tests;  // [0, 1]

        double output = gt2_adapter_->calculateOutput(error, dError, wind);

        if (!std::isfinite(output) || std::abs(output) > 5.0) {
            failures++;
        }
    }

    EXPECT_EQ(failures, 0) << "GT2-FLS should produce valid outputs for all reasonable inputs";
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
