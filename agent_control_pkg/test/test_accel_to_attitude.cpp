#include "agent_control_pkg/core/accel_to_attitude.hpp"
#include <cmath>
#include <gtest/gtest.h>

using namespace agent_control_pkg::core;

class AccelToAttitudeTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Default Crazyflie-like config
    config_.gravity = 9.81;
    config_.mass = 0.027;  // 27g
    config_.hover_thrust = config_.mass * config_.gravity;
    config_.max_thrust = 0.6;
    config_.min_thrust = 0.0;
    config_.max_roll_rad = 0.5236;   // 30 degrees
    config_.max_pitch_rad = 0.5236;  // 30 degrees
    config_.normalize_thrust = true;

    converter_ = std::make_unique<AccelToAttitude>(config_);
  }

  AccelToAttitudeConfig config_;
  std::unique_ptr<AccelToAttitude> converter_;

  // Helper: degrees to radians
  static double deg2rad(double deg) { return deg * M_PI / 180.0; }

  // Helper: radians to degrees
  static double rad2deg(double rad) { return rad * 180.0 / M_PI; }
};

// Test default config initialization
TEST_F(AccelToAttitudeTest, DefaultConfigValues) {
  AccelToAttitudeConfig default_config;
  EXPECT_DOUBLE_EQ(default_config.gravity, 9.81);
  EXPECT_DOUBLE_EQ(default_config.mass, 0.027);
  EXPECT_DOUBLE_EQ(default_config.max_roll_rad, 0.5236);
  EXPECT_DOUBLE_EQ(default_config.max_pitch_rad, 0.5236);
  EXPECT_TRUE(default_config.normalize_thrust);
}

// Test hover condition: zero acceleration should produce zero angles
TEST_F(AccelToAttitudeTest, HoverCondition) {
  auto cmd = converter_->convert(0.0, 0.0, 0.0);

  EXPECT_NEAR(cmd.roll, 0.0, 1e-10);
  EXPECT_NEAR(cmd.pitch, 0.0, 1e-10);
  // Thrust at hover should equal mass * g
  EXPECT_NEAR(cmd.thrust, config_.hover_thrust, 1e-6);
}

// Test pitch from positive X acceleration
// ax = g * tan(theta) => theta = atan(ax / g)
TEST_F(AccelToAttitudeTest, PositiveXAccelGivesPitch) {
  const double ax = 2.0;  // m/s^2
  auto cmd = converter_->convert(ax, 0.0, 0.0);

  // Expected pitch = atan(ax / g)
  const double expected_pitch = std::atan2(ax, config_.gravity);
  EXPECT_NEAR(cmd.pitch, expected_pitch, 1e-6);
  EXPECT_NEAR(cmd.roll, 0.0, 1e-10);
}

// Test roll from positive Y acceleration
// ay = -g * tan(phi) => phi = atan(-ay / g)
TEST_F(AccelToAttitudeTest, PositiveYAccelGivesNegativeRoll) {
  const double ay = 2.0;  // m/s^2 (positive = left)
  auto cmd = converter_->convert(0.0, ay, 0.0);

  // Expected roll = atan(-ay / g)
  const double expected_roll = std::atan2(-ay, config_.gravity);
  EXPECT_NEAR(cmd.roll, expected_roll, 1e-6);
  EXPECT_NEAR(cmd.pitch, 0.0, 1e-10);
}

// Test negative X acceleration gives negative pitch
TEST_F(AccelToAttitudeTest, NegativeXAccelGivesNegativePitch) {
  const double ax = -3.0;
  auto cmd = converter_->convert(ax, 0.0, 0.0);

  const double expected_pitch = std::atan2(ax, config_.gravity);
  EXPECT_NEAR(cmd.pitch, expected_pitch, 1e-6);
  EXPECT_LT(cmd.pitch, 0.0);  // Should be negative
}

// Test negative Y acceleration gives positive roll
TEST_F(AccelToAttitudeTest, NegativeYAccelGivesPositiveRoll) {
  const double ay = -2.0;  // m/s^2 (negative = right)
  auto cmd = converter_->convert(0.0, ay, 0.0);

  const double expected_roll = std::atan2(-ay, config_.gravity);
  EXPECT_NEAR(cmd.roll, expected_roll, 1e-6);
  EXPECT_GT(cmd.roll, 0.0);  // Should be positive
}

// Test attitude angle clamping at limits
TEST_F(AccelToAttitudeTest, AngleClamping) {
  // Request acceleration that would require > 30 deg pitch
  // tan(30 deg) * g ~= 5.66 m/s^2
  const double extreme_ax = 15.0;  // Way beyond limit
  auto cmd = converter_->convert(extreme_ax, 0.0, 0.0);

  // Pitch should be clamped to max_pitch_rad
  EXPECT_NEAR(cmd.pitch, config_.max_pitch_rad, 1e-6);

  // Same for roll
  const double extreme_ay = 15.0;
  cmd = converter_->convert(0.0, extreme_ay, 0.0);
  EXPECT_NEAR(std::abs(cmd.roll), config_.max_roll_rad, 1e-6);
}

// Test thrust computation for upward acceleration
TEST_F(AccelToAttitudeTest, UpwardAccelerationIncreasesThrust) {
  const double az = 2.0;  // m/s^2 upward
  auto cmd = converter_->convert(0.0, 0.0, az);

  // thrust = mass * (az + g)
  const double expected_thrust = config_.mass * (az + config_.gravity);
  EXPECT_NEAR(cmd.thrust, expected_thrust, 1e-6);
  EXPECT_GT(cmd.thrust, config_.hover_thrust);
}

// Test thrust computation for downward acceleration
TEST_F(AccelToAttitudeTest, DownwardAccelerationDecreasesThrust) {
  const double az = -2.0;  // m/s^2 downward
  auto cmd = converter_->convert(0.0, 0.0, az);

  const double expected_thrust = config_.mass * (az + config_.gravity);
  EXPECT_NEAR(cmd.thrust, expected_thrust, 1e-6);
  EXPECT_LT(cmd.thrust, config_.hover_thrust);
}

// Test thrust clamping at maximum
TEST_F(AccelToAttitudeTest, ThrustClampingMax) {
  const double extreme_az = 100.0;  // Very high upward accel
  auto cmd = converter_->convert(0.0, 0.0, extreme_az);

  EXPECT_NEAR(cmd.thrust, config_.max_thrust, 1e-6);
  EXPECT_NEAR(cmd.thrust_normalized, 1.0, 1e-6);
}

// Test thrust clamping at minimum
TEST_F(AccelToAttitudeTest, ThrustClampingMin) {
  const double extreme_az = -20.0;  // Extreme downward
  auto cmd = converter_->convert(0.0, 0.0, extreme_az);

  EXPECT_NEAR(cmd.thrust, config_.min_thrust, 1e-6);
  EXPECT_NEAR(cmd.thrust_normalized, 0.0, 1e-6);
}

// Test thrust normalization
TEST_F(AccelToAttitudeTest, ThrustNormalization) {
  auto cmd = converter_->convert(0.0, 0.0, 0.0);

  // At hover, normalized thrust should be hover_thrust / max_thrust
  const double expected_normalized = config_.hover_thrust / config_.max_thrust;
  EXPECT_NEAR(cmd.thrust_normalized, expected_normalized, 1e-6);
}

// Test combined XY acceleration
TEST_F(AccelToAttitudeTest, CombinedXYAcceleration) {
  const double ax = 1.5;
  const double ay = -1.0;
  auto cmd = converter_->convert(ax, ay, 0.0);

  // Both roll and pitch should be non-zero
  EXPECT_GT(std::abs(cmd.pitch), 1e-6);
  EXPECT_GT(std::abs(cmd.roll), 1e-6);

  // Verify directions
  EXPECT_GT(cmd.pitch, 0.0);  // Positive ax -> positive pitch
  EXPECT_GT(cmd.roll, 0.0);   // Negative ay -> positive roll
}

// Test yaw passthrough
TEST_F(AccelToAttitudeTest, YawPassthrough) {
  const double yaw_cmd = 0.5;  // radians
  auto cmd = converter_->convert(0.0, 0.0, 0.0, yaw_cmd);

  EXPECT_DOUBLE_EQ(cmd.yaw, yaw_cmd);
}

// Test thrust compensation for tilted attitude
TEST_F(AccelToAttitudeTest, TiltThrustCompensation) {
  // When pitched/rolled, vertical thrust component decreases
  // Thrust should increase to compensate
  const double ax = 3.0;
  auto cmd_tilted = converter_->convert(ax, 0.0, 0.0);
  auto cmd_level = converter_->convert(0.0, 0.0, 0.0);

  // Tilted thrust should be higher to maintain same vertical force
  EXPECT_GT(cmd_tilted.thrust, cmd_level.thrust);
}

// Test config update
TEST_F(AccelToAttitudeTest, ConfigUpdate) {
  AccelToAttitudeConfig new_config;
  new_config.mass = 0.05;  // 50g
  new_config.max_thrust = 1.0;
  new_config.gravity = 9.81;

  converter_->setConfig(new_config);

  auto cmd = converter_->convert(0.0, 0.0, 0.0);

  // New hover thrust = 0.05 * 9.81 ~= 0.4905
  const double expected_hover = 0.05 * 9.81;
  EXPECT_NEAR(cmd.thrust, expected_hover, 1e-6);
}

// Test small angle approximation validity
// For small angles: sin(theta) ~= theta, tan(theta) ~= theta
TEST_F(AccelToAttitudeTest, SmallAngleApproximation) {
  // For small accelerations, pitch ~= ax / g
  const double ax_small = 0.5;  // Small acceleration
  auto cmd = converter_->convert(ax_small, 0.0, 0.0);

  // atan(0.5/9.81) ~= 0.0509 rad, linear approx = 0.5/9.81 = 0.051
  const double linear_approx = ax_small / config_.gravity;
  const double exact = std::atan2(ax_small, config_.gravity);

  // Our implementation should use exact (atan), but they should be close
  EXPECT_NEAR(cmd.pitch, exact, 1e-10);
  // And both should be close for small angles
  EXPECT_NEAR(linear_approx, exact, 0.001);
}
