// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "smart_bringup/imu_fusion.hpp"

using smart_bringup::ImuFusion;
using smart_bringup::ImuFusionConfig;

namespace
{
double yaw(const ImuFusion::Quaternion & q)
{
  return std::atan2(
    2.0 * (q[0] * q[3] + q[1] * q[2]),
    1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}
}  // namespace

TEST(ImuFusion, StationaryBiasAndNormalizedOrientation)
{
  ImuFusion fusion;
  const ImuFusion::Vector bias{0.057, -0.008, -0.022};
  for (int i = 0; i < 300; ++i) {
    const bool ready = fusion.update({0, 0, 11.0}, bias, 1.0 + i * 0.01);
    if (i < 200) {EXPECT_FALSE(ready);}
  }
  ASSERT_TRUE(fusion.calibrated());
  double norm = 0.0;
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(fusion.bias()[i], bias[i], 1e-10);
    EXPECT_NEAR(fusion.angular_velocity()[i], 0.0, 1e-10);
  }
  for (double value : fusion.orientation()) {
    norm += value * value;
  }
  EXPECT_NEAR(norm, 1.0, 1e-10);
  EXPECT_NEAR(yaw(fusion.orientation()), 0.0, 1e-6);
}

TEST(ImuFusion, TiltedBodyRatesAndRelativeYaw)
{
  ImuFusionConfig config;
  config.calibration_seconds = 0.0;
  ImuFusion fusion(config);
  const double roll = M_PI / 6.0, pitch = M_PI / 9.0;
  const ImuFusion::Vector gravity{
    -9.80665 * std::sin(pitch),
    9.80665 * std::sin(roll) * std::cos(pitch),
    9.80665 * std::cos(roll) * std::cos(pitch)};
  ImuFusion::Vector rates{};
  for (std::size_t i = 0; i < 3; ++i) {
    rates[i] = gravity[i] * 0.5 / 9.80665;
  }
  ASSERT_TRUE(fusion.update(gravity, {0, 0, 0}, 1.0));
  for (int i = 1; i <= 200; ++i) {
    ASSERT_TRUE(fusion.update(gravity, rates, 1.0 + i * 0.01));
  }
  EXPECT_NEAR(yaw(fusion.orientation()), 1.0, 0.005);
}

TEST(ImuFusion, MotionAndInvalidSamplesRestartCalibration)
{
  ImuFusionConfig config;
  config.calibration_seconds = 0.2;
  config.min_calibration_samples = 20;
  ImuFusion fusion(config);
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(fusion.update({0, 0, 9.8}, {0, 0, 0.5}, 1.0 + i * 0.01));
  }
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(fusion.update({0, 0, 9.8}, {0.01, 0, 0}, 2.0 + i * 0.01));
  }
  EXPECT_FALSE(fusion.update({0, 0, 9.8}, {NAN, 0, 0}, 2.1));
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(fusion.update({0, 0, 9.8}, {0.02, 0, 0}, 2.11 + i * 0.01));
  }
  for (int i = 0; i < 50; ++i) {
    fusion.update({0, 0, 9.8}, {0.02, 0, 0}, 2.21 + i * 0.01);
  }
  ASSERT_TRUE(fusion.calibrated());
  EXPECT_NEAR(fusion.bias()[0], 0.02, 1e-10);
}

TEST(ImuFusion, GapAndBackwardTimeResetRelativeYaw)
{
  ImuFusionConfig config;
  config.calibration_seconds = 0.0;
  ImuFusion fusion(config);
  ASSERT_TRUE(fusion.update({0, 0, 9.8}, {0, 0, 0}, 1.0));
  for (int i = 1; i <= 100; ++i) {
    fusion.update({0, 0, 9.8}, {0, 0, 1}, 1.0 + i * 0.01);
  }
  EXPECT_NEAR(yaw(fusion.orientation()), 1.0, 0.005);
  ASSERT_TRUE(fusion.update({0, 0, 9.8}, {0, 0, 1}, 3.0));
  EXPECT_TRUE(fusion.reinitialized());
  EXPECT_NEAR(yaw(fusion.orientation()), 0.0, 1e-10);
  ASSERT_TRUE(fusion.update({0, 0, 9.8}, {0, 0, 1}, 2.0));
  EXPECT_TRUE(fusion.reinitialized());
  EXPECT_NEAR(yaw(fusion.orientation()), 0.0, 1e-10);
}

TEST(ImuFusion, FreefallAndInvalidSamplesNeverPublishInvalidQuaternion)
{
  ImuFusionConfig config;
  config.calibration_seconds = 0.0;
  ImuFusion fusion(config);
  EXPECT_FALSE(fusion.update({0, 0, 0}, {0, 0, 0}, 1.0));
  EXPECT_FALSE(fusion.update({0, 0, INFINITY}, {0, 0, 0}, 1.01));
  ASSERT_TRUE(fusion.update({0, 0, 9.8}, {0, 0, 0}, 1.02));
  EXPECT_FALSE(fusion.update({0, 0, 9.8}, {NAN, 0, 0}, 1.03));
  ASSERT_TRUE(fusion.update({0, 0, 0}, {0, 0, 1}, 1.04));
  for (double value : fusion.orientation()) {
    EXPECT_TRUE(std::isfinite(value));
  }
  fusion.reset();
  ASSERT_TRUE(fusion.update({0, 0, 9.8}, {0, 0, 0}, 2.0));
  EXPECT_NEAR(yaw(fusion.orientation()), 0.0, 1e-10);
}

TEST(ImuFusion, InvalidParametersAreRejected)
{
  ImuFusionConfig config;
  config.gain = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ImuFusion{config}, std::invalid_argument);
  config.gain = 0.1;
  config.max_sample_gap = 0.0;
  EXPECT_THROW(ImuFusion{config}, std::invalid_argument);
}
