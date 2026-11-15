// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef SMART_BRINGUP__IMU_FUSION_HPP_
#define SMART_BRINGUP__IMU_FUSION_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "imu_filter_madgwick/imu_filter.h"

namespace smart_bringup
{

struct ImuFusionConfig
{
  double calibration_seconds{2.0};
  double gain{0.1};
  double max_sample_gap{0.2};
  double stationary_gyro_threshold{0.15};
  std::size_t min_calibration_samples{50};
};

class ImuFusion
{
public:
  using Vector = std::array<double, 3>;
  using Quaternion = std::array<double, 4>;  // w, x, y, z

  explicit ImuFusion(const ImuFusionConfig & config = ImuFusionConfig())
  : config_(config)
  {
    if (!std::isfinite(config_.calibration_seconds) || config_.calibration_seconds < 0.0 ||
      !std::isfinite(config_.gain) || config_.gain < 0.0 || config_.gain > 1.0 ||
      !std::isfinite(config_.max_sample_gap) || config_.max_sample_gap <= 0.0 ||
      !std::isfinite(config_.stationary_gyro_threshold) ||
      config_.stationary_gyro_threshold <= 0.0 || config_.min_calibration_samples < 2)
    {
      throw std::invalid_argument("invalid IMU fusion configuration");
    }
    filter_.setWorldFrame(WorldFrame::ENU);
    filter_.setAlgorithmGain(config_.gain);
    filter_.setDriftBiasGain(0.0);
    reset();
  }

  void reset()
  {
    calibrated_ = config_.calibration_seconds == 0.0;
    initialized_ = false;
    have_stamp_ = false;
    bias_.fill(0.0);
    restart_calibration();
  }

  bool calibrated() const {return calibrated_;}
  const Vector & bias() const {return bias_;}
  const Vector & angular_velocity() const {return corrected_gyro_;}
  const Quaternion & orientation() const {return quaternion_;}
  bool reinitialized() const {return reinitialized_;}

  bool update(const Vector & acceleration, const Vector & gyro, double stamp)
  {
    reinitialized_ = false;
    if (!finite(acceleration) || !finite(gyro) || !std::isfinite(stamp)) {
      if (!calibrated_) {restart_calibration();}
      return false;
    }
    const double dt = have_stamp_ ? stamp - previous_stamp_ : 0.0;
    const bool discontinuity = have_stamp_ && (dt <= 0.0 || dt > config_.max_sample_gap);
    previous_stamp_ = stamp;
    have_stamp_ = true;
    if (discontinuity) {
      initialized_ = false;
      if (!calibrated_) {restart_calibration();}
    }
    const double norm = magnitude(acceleration);
    const bool gravity_valid = norm >= 6.0 && norm <= 15.0;
    if (!calibrated_) {
      if (!gravity_valid || magnitude(gyro) > config_.stationary_gyro_threshold) {
        restart_calibration();
        return false;
      }
      Vector unit{};
      for (std::size_t i = 0; i < 3; ++i) {unit[i] = acceleration[i] / norm;}
      if (samples_ > 0 &&
        (difference_squared(unit, reference_gravity_) > 0.01 ||
        difference_squared(gyro, reference_gyro_) > 0.0004))
      {
        restart_calibration();
      }
      if (samples_ == 0) {
        calibration_start_ = stamp;
        reference_gravity_ = unit;
        reference_gyro_ = gyro;
      }
      ++samples_;
      for (std::size_t i = 0; i < 3; ++i) {gyro_sum_[i] += gyro[i];}
      if (stamp - calibration_start_ < config_.calibration_seconds ||
        samples_ < config_.min_calibration_samples)
      {
        return false;
      }
      for (std::size_t i = 0; i < 3; ++i) {bias_[i] = gyro_sum_[i] / samples_;}
      calibrated_ = true;
    }
    for (std::size_t i = 0; i < 3; ++i) {corrected_gyro_[i] = gyro[i] - bias_[i];}
    if (!initialized_) {
      if (!gravity_valid) {return false;}
      // Gravity initializes tilt; yaw starts at zero in a relative ENU world.
      const double roll = std::atan2(acceleration[1], acceleration[2]);
      const double pitch =
        std::atan2(-acceleration[0], std::hypot(acceleration[1], acceleration[2]));
      const double cr = std::cos(roll / 2.0), sr = std::sin(roll / 2.0);
      const double cp = std::cos(pitch / 2.0), sp = std::sin(pitch / 2.0);
      filter_.setOrientation(cr * cp, sr * cp, cr * sp, -sr * sp);
      initialized_ = true;
      reinitialized_ = true;
    } else {
      // Reject implausible acceleration as a gravity reference, but keep gyro integration.
      filter_.madgwickAHRSupdateIMU(
        corrected_gyro_[0], corrected_gyro_[1], corrected_gyro_[2],
        gravity_valid ? acceleration[0] : 0.0,
        gravity_valid ? acceleration[1] : 0.0,
        gravity_valid ? acceleration[2] : 0.0, dt);
    }
    filter_.getOrientation(quaternion_[0], quaternion_[1], quaternion_[2], quaternion_[3]);
    for (double value : quaternion_) {
      if (!std::isfinite(value)) {initialized_ = false; return false;}
    }
    return true;
  }

private:
  static bool finite(const Vector & values)
  {
    return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
  }

  static double magnitude(const Vector & values)
  {
    return std::hypot(std::hypot(values[0], values[1]), values[2]);
  }

  static double difference_squared(const Vector & left, const Vector & right)
  {
    double sum = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {sum += (left[i] - right[i]) * (left[i] - right[i]);}
    return sum;
  }

  void restart_calibration()
  {
    samples_ = 0;
    gyro_sum_.fill(0.0);
  }

  ImuFusionConfig config_;
  ImuFilter filter_;
  Vector bias_{}, corrected_gyro_{}, gyro_sum_{}, reference_gravity_{}, reference_gyro_{};
  Quaternion quaternion_{1.0, 0.0, 0.0, 0.0};
  bool calibrated_{false}, initialized_{false}, have_stamp_{false}, reinitialized_{false};
  double previous_stamp_{0.0}, calibration_start_{0.0};
  std::size_t samples_{0};
};

}  // namespace smart_bringup

#endif  // SMART_BRINGUP__IMU_FUSION_HPP_
