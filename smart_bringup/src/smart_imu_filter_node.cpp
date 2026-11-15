// SPDX-License-Identifier: GPL-3.0-or-later
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "smart_bringup/imu_fusion.hpp"

namespace smart_bringup
{

class SmartImuFilter : public rclcpp::Node
{
public:
  SmartImuFilter()
  : Node("smart_imu_filter")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    ImuFusionConfig config;
    config.calibration_seconds = declare_parameter<double>("calibration_seconds", 2.0, descriptor);
    config.gain = declare_parameter<double>("gain", 0.1, descriptor);
    config.max_sample_gap = declare_parameter<double>("max_sample_gap", 0.2, descriptor);
    config.stationary_gyro_threshold = declare_parameter<double>(
      "stationary_gyro_threshold", 0.15, descriptor);
    const int samples = declare_parameter<int>("min_calibration_samples", 50, descriptor);
    if (samples < 2) {throw std::invalid_argument("min_calibration_samples must be >= 2");}
    config.min_calibration_samples = static_cast<std::size_t>(samples);
    fusion_ = std::make_unique<ImuFusion>(config);
    const double stddev = declare_parameter<double>("orientation_stddev", 0.0, descriptor);
    if (!std::isfinite(stddev) || stddev < 0.0 || stddev > 1.0) {
      throw std::invalid_argument("orientation_stddev must be finite and in [0, 1] radians");
    }
    orientation_variance_ = stddev * stddev;
    const auto input = declare_parameter<std::string>("input_topic", "imu/data_raw", descriptor);
    const auto output = declare_parameter<std::string>("output_topic", "imu/data", descriptor);
    if (get_node_topics_interface()->resolve_topic_name(input) ==
      get_node_topics_interface()->resolve_topic_name(output))
    {
      throw std::invalid_argument("IMU input and output topics must differ");
    }
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output, 10);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input, rclcpp::SensorDataQoS(),
      std::bind(&SmartImuFilter::on_imu, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::seconds(2), [this]() {
        if (!have_message_ ||
        std::chrono::steady_clock::now() - last_receive_ > std::chrono::milliseconds(500))
        {
          RCLCPP_WARN(get_logger(), "Waiting for fresh fused IMU data; calibration may be pending");
        }
      });
    RCLCPP_INFO(
      get_logger(), "Madgwick six-axis fusion: %s -> %s; keep IMU still for %.1f s. "
      "Yaw is relative and can drift; this node publishes no TF or motor commands.",
      input.c_str(), output.c_str(), config.calibration_seconds);
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr raw)
  {
    if (raw->header.frame_id.empty() || raw->header.stamp.sec < 0 ||
      raw->header.stamp.nanosec >= 1000000000U ||
      (raw->header.stamp.sec == 0 && raw->header.stamp.nanosec == 0))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 2000, "Rejecting IMU with missing frame/stamp");
      return;
    }
    if (!frame_.empty() && frame_ != raw->header.frame_id) {
      fusion_->reset();
      bias_reported_ = false;
      RCLCPP_WARN(get_logger(), "IMU frame changed; restarting stationary bias calibration");
    }
    frame_ = raw->header.frame_id;
    const ImuFusion::Vector acceleration{
      raw->linear_acceleration.x, raw->linear_acceleration.y, raw->linear_acceleration.z};
    const ImuFusion::Vector gyro{
      raw->angular_velocity.x, raw->angular_velocity.y, raw->angular_velocity.z};
    if (!fusion_->update(acceleration, gyro, rclcpp::Time(raw->header.stamp).seconds())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 2000, "No fused sample: calibrating or rejecting invalid IMU data; "
        "keep IMU still during startup");
      return;
    }
    have_message_ = true;
    last_receive_ = std::chrono::steady_clock::now();
    if (!bias_reported_) {
      const auto & bias = fusion_->bias();
      RCLCPP_INFO(
        get_logger(), "IMU bias ready [%.6f, %.6f, %.6f] rad/s", bias[0], bias[1], bias[2]);
      bias_reported_ = true;
    }
    if (fusion_->reinitialized()) {
      RCLCPP_INFO(get_logger(), "IMU tilt initialized; relative yaw reset to zero");
    }
    auto output = *raw;
    const auto & q = fusion_->orientation();
    output.orientation.w = q[0]; output.orientation.x = q[1];
    output.orientation.y = q[2]; output.orientation.z = q[3];
    output.orientation_covariance.fill(0.0);
    for (const auto index : {0, 4, 8}) {
      output.orientation_covariance[index] = orientation_variance_;
    }
    const auto & gyro_corrected = fusion_->angular_velocity();
    output.angular_velocity.x = gyro_corrected[0];
    output.angular_velocity.y = gyro_corrected[1];
    output.angular_velocity.z = gyro_corrected[2];
    publisher_->publish(output);
  }

  std::unique_ptr<ImuFusion> fusion_;
  double orientation_variance_{0.0};
  bool bias_reported_{false}, have_message_{false};
  std::string frame_;
  std::chrono::steady_clock::time_point last_receive_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace smart_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<smart_bringup::SmartImuFilter>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("smart_imu_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
