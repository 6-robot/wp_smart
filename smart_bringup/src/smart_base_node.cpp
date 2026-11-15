#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "smart_bringup/protocol.hpp"

namespace smart_bringup
{
namespace
{

using namespace std::chrono_literals;

speed_t baud_to_termios(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: throw std::invalid_argument("unsupported baud rate");
  }
}

double steady_seconds(std::chrono::steady_clock::time_point time)
{
  return std::chrono::duration<double>(time.time_since_epoch()).count();
}

void set_covariance_diagonal(
  std::array<double, 36> & covariance, const std::vector<double> & diagonal)
{
  covariance.fill(0.0);
  constexpr std::array<std::size_t, 6> indices = {0, 7, 14, 21, 28, 35};
  for (std::size_t i = 0; i < indices.size(); ++i) {
    covariance[indices[i]] = diagonal[i];
  }
}

}  // namespace

class SmartBaseNode : public rclcpp::Node
{
public:
  SmartBaseNode()
  : Node("smart_base_node"),
    port_(declare_parameter<std::string>(
        "port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C85476222-if00")),
    baud_rate_(declare_parameter<int>("baud_rate", 460800)),
    wheel_diameter_(declare_parameter<double>("wheel_diameter", 0.0657)),
    wheel_base_(declare_parameter<double>("wheel_base", 0.1805)),
    counts_per_wheel_rev_(declare_parameter<int64_t>("encoder_counts_per_wheel_rev", 262144)),
    left_encoder_sign_(declare_parameter<int>("left_encoder_sign", 1)),
    right_encoder_sign_(declare_parameter<int>("right_encoder_sign", -1)),
    command_rate_(declare_parameter<double>("command_rate", 50.0)),
    cmd_vel_timeout_(declare_parameter<double>("cmd_vel_timeout", 3.0)),
    enable_cmd_vel_timeout_(declare_parameter<bool>(
        "enable_cmd_vel_timeout", false,
        rcl_interfaces::msg::ParameterDescriptor().set__read_only(true))),
    status_timeout_(declare_parameter<double>("status_timeout", 0.2)),
    reconnect_interval_(declare_parameter<double>("reconnect_interval", 1.0)),
    max_linear_velocity_(declare_parameter<double>("max_linear_velocity", 0.5)),
    max_angular_velocity_(declare_parameter<double>("max_angular_velocity", 2.5)),
    max_encoder_delta_(declare_parameter<int>("max_encoder_delta", 256000)),
    publish_tf_(declare_parameter<bool>("publish_tf", true)),
    odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
    base_frame_(declare_parameter<std::string>("base_frame", "base_footprint")),
    battery_frame_(declare_parameter<std::string>("battery_frame", "base_link")),
    imu_frame_(declare_parameter<std::string>("imu_frame", "imu_link")),
    laser_frame_(declare_parameter<std::string>("laser_frame", "laser_scan_link")),
    scan_bins_(declare_parameter<int>("scan_bins", 450)),
    scan_range_min_(declare_parameter<double>("scan_range_min", 0.24)),
    scan_range_max_(declare_parameter<double>("scan_range_max", 10.0)),
    scan_angle_offset_(declare_parameter<double>("scan_angle_offset_deg", 0.0)),
    scan_angle_direction_(declare_parameter<double>("scan_angle_direction", -1.0)),
    require_sensors_(declare_parameter<bool>("require_sensors", true)),
    pose_covariance_diagonal_(declare_parameter<std::vector<double>>(
        "pose_covariance_diagonal", {0.02, 0.02, 1.0e6, 1.0e6, 1.0e6, 0.05})),
    twist_covariance_diagonal_(declare_parameter<std::vector<double>>(
        "twist_covariance_diagonal", {0.03, 0.03, 1.0e6, 1.0e6, 1.0e6, 0.08})),
    odometry_(
      wheel_diameter_, wheel_base_, counts_per_wheel_rev_, left_encoder_sign_,
      right_encoder_sign_, max_encoder_delta_),
    updater_(this)
  {
    validate_parameters();

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 20);
    battery_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
      "battery_state", rclcpp::SensorDataQoS());
    imu_publisher_ =
      create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());
    scan_publisher_ =
      create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    encoder_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>("encoder_counts", 20);
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 20,
      std::bind(&SmartBaseNode::cmd_vel_callback, this, std::placeholders::_1));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    updater_.setHardwareID("smart_base");
    updater_.add("Chassis communication", this, &SmartBaseNode::produce_diagnostics);

    running_.store(true);
    io_thread_ = std::thread(&SmartBaseNode::io_loop, this);
    RCLCPP_INFO(
      get_logger(), "Smart base driver configured for %s at %d baud", port_.c_str(), baud_rate_);
  }

  ~SmartBaseNode() override
  {
    running_.store(false);
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
  }

private:
  struct DriverState
  {
    bool connected{false};
    bool have_status{false};
    RobotStatus status;
    std::chrono::steady_clock::time_point last_status_time{};
    uint64_t parser_bad_checksum{0};
    uint64_t parser_bad_length{0};
    uint64_t invalid_status{0};
    uint64_t invalid_sensors{0};
    uint64_t imu_samples{0};
    uint64_t scans{0};
    uint64_t dropped_scans{0};
    std::chrono::steady_clock::time_point last_imu_time{}, last_scan_time{};
    uint64_t tx_frames{0};
    uint64_t reconnects{0};
  };

  void validate_parameters()
  {
    if (command_rate_ <= 0.0 || cmd_vel_timeout_ <= 0.0 || status_timeout_ <= 0.0 ||
      reconnect_interval_ <= 0.0 || max_linear_velocity_ <= 0.0 ||
      max_angular_velocity_ <= 0.0)
    {
      throw std::invalid_argument("rates, timeouts, and velocity limits must be positive");
    }
    if (pose_covariance_diagonal_.size() != 6U || twist_covariance_diagonal_.size() != 6U) {
      throw std::invalid_argument("covariance diagonal parameters must contain six values");
    }
    if (scan_bins_ < 2 || scan_bins_ > 3600 || !std::isfinite(scan_range_min_) ||
      !std::isfinite(scan_range_max_) || scan_range_min_ < 0 ||
      scan_range_max_ <= scan_range_min_ || !std::isfinite(scan_angle_offset_) ||
      (scan_angle_direction_ != 1.0 && scan_angle_direction_ != -1.0))
    {
      throw std::invalid_argument("invalid scan geometry");
    }
    static_cast<void>(baud_to_termios(baud_rate_));
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite cmd_vel");
      return;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    command_linear_ = std::clamp(message->linear.x, -max_linear_velocity_, max_linear_velocity_);
    command_angular_ = std::clamp(
      message->angular.z, -max_angular_velocity_, max_angular_velocity_);
    last_command_time_ = std::chrono::steady_clock::now();
    have_command_ = true;
    ++command_sequence_;
  }

  bool open_serial()
  {
    const int fd = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN(get_logger(), "Cannot open %s: %s", port_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr(%s) failed: %s", port_.c_str(), std::strerror(errno));
      close(fd);
      return false;
    }

    cfmakeraw(&tty);
    const speed_t baud = baud_to_termios(baud_rate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr(%s) failed: %s", port_.c_str(), std::strerror(errno));
      close(fd);
      return false;
    }

    tcflush(fd, TCIOFLUSH);
    serial_fd_ = fd;
    parser_.reset();
    scan_assembler_.reset();
    have_scan_stamp_ = false;
    odometry_.reset_baseline();
    {
      std::lock_guard<std::mutex> command_lock(command_mutex_);
      command_linear_ = 0.0;
      command_angular_ = 0.0;
      have_command_ = false;
      ++command_sequence_;
    }
    last_command_sent_time_ = {};
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      state_.connected = true;
      state_.have_status = false;
      state_.last_imu_time = {};
      state_.last_scan_time = {};
      ++state_.reconnects;
    }
    RCLCPP_INFO(get_logger(), "Connected to smart base on %s", port_.c_str());
    return true;
  }

  void close_serial()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
    parser_.reset();
    scan_assembler_.reset();
    have_scan_stamp_ = false;
    odometry_.reset_baseline();
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.connected = false;
    state_.have_status = false;
  }

  bool write_bytes(const std::vector<uint8_t> & bytes)
  {
    std::size_t offset = 0;
    const auto deadline = std::chrono::steady_clock::now() + 20ms;
    while (offset < bytes.size() && std::chrono::steady_clock::now() < deadline) {
      const ssize_t written = write(serial_fd_, bytes.data() + offset, bytes.size() - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
      } else if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        RCLCPP_WARN(get_logger(), "Serial write failed: %s", std::strerror(errno));
        return false;
      } else {
        pollfd descriptor{serial_fd_, POLLOUT, 0};
        static_cast<void>(poll(&descriptor, 1, 2));
      }
    }
    return offset == bytes.size();
  }

  bool send_command(std::chrono::steady_clock::time_point now)
  {
    double linear = 0.0;
    double angular = 0.0;
    uint64_t sequence;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      sequence = command_sequence_;
      if (!enable_cmd_vel_timeout_ &&
        last_command_sent_time_.time_since_epoch().count() != 0 &&
        sequence == last_sent_command_sequence_ && now - last_command_sent_time_ < 1s)
      {
        return true;
      }
      if (have_command_ &&
        (!enable_cmd_vel_timeout_ ||
        std::chrono::duration<double>(now - last_command_time_).count() <= cmd_vel_timeout_))
      {
        linear = command_linear_;
        angular = command_angular_;
      }
    }

    if (!write_bytes(make_cmd_vel(static_cast<float>(linear), static_cast<float>(angular)))) {
      return false;
    }
    last_command_sent_time_ = now;
    last_sent_command_sequence_ = sequence;
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++state_.tx_frames;
    return true;
  }

  void io_loop()
  {
    const double poll_rate = enable_cmd_vel_timeout_ ? command_rate_ : std::max(command_rate_, 1.0);
    const auto command_period = std::chrono::duration<double>(1.0 / poll_rate);
    auto next_reconnect = std::chrono::steady_clock::now();
    auto next_command = std::chrono::steady_clock::now();
    std::array<uint8_t, 512> read_buffer{};

    while (running_.load() && rclcpp::ok()) {
      const auto now = std::chrono::steady_clock::now();
      if (serial_fd_ < 0) {
        if (now < next_reconnect) {
          std::this_thread::sleep_for(10ms);
          continue;
        }
        if (!open_serial()) {
          next_reconnect = now +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(reconnect_interval_));
          continue;
        }
        next_command = now;
      }

      if (now >= next_command) {
        if (!send_command(now)) {
          close_serial();
          next_reconnect = now +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(reconnect_interval_));
          continue;
        }
        do {
          next_command +=
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(command_period);
        } while (next_command <= now);
      }

      pollfd descriptor{serial_fd_, POLLIN, 0};
      const int poll_result = poll(&descriptor, 1, 2);
      if (poll_result < 0 && errno != EINTR) {
        RCLCPP_WARN(get_logger(), "Serial poll failed: %s", std::strerror(errno));
        close_serial();
        continue;
      }
      if (poll_result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        RCLCPP_WARN(get_logger(), "Serial device disconnected");
        close_serial();
        next_reconnect = now +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(reconnect_interval_));
        continue;
      }
      if (poll_result <= 0 || (descriptor.revents & POLLIN) == 0) {
        continue;
      }

      const ssize_t count = read(serial_fd_, read_buffer.data(), read_buffer.size());
      if (count > 0) {
        process_serial_data(read_buffer.data(), static_cast<std::size_t>(count));
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        RCLCPP_WARN(get_logger(), "Serial read failed: %s", std::strerror(errno));
        close_serial();
      }
    }

    if (serial_fd_ >= 0) {
      const auto stop_frame = make_cmd_vel(0.0F, 0.0F);
      for (int i = 0; i < 5; ++i) {
        static_cast<void>(write_bytes(stop_frame));
        std::this_thread::sleep_for(20ms);
      }
    }
    close_serial();
  }

  void process_serial_data(const uint8_t * data, std::size_t size)
  {
    const auto frames = parser_.feed(data, size);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_.parser_bad_checksum = parser_.bad_checksum_count();
      state_.parser_bad_length = parser_.bad_length_count();
    }

    for (const auto & frame : frames) {
      if (frame.type == kMsgStatus) {
        RobotStatus status;
        if (decode_status(frame.payload, status)) {
          process_status(status);
        } else {
          std::lock_guard<std::mutex> lock(state_mutex_);
          ++state_.invalid_status;
        }
      } else if (frame.type == kMsgImu) {
        ImuSample sample;
        if (decode_imu(frame.payload, sample)) {
          publish_imu(sample);
        } else {
          std::lock_guard<std::mutex> lock(state_mutex_);
          ++state_.invalid_sensors;
        }
      } else if (frame.type == kMsgScanFragment) {
        ScanFragment fragment;
        CompleteScan scan;
        if (!decode_scan_fragment(frame.payload, fragment)) {
          scan_assembler_.reset();
          std::lock_guard<std::mutex> lock(state_mutex_);
          ++state_.invalid_sensors;
        } else if (scan_assembler_.add(fragment, scan)) {
          publish_scan(scan);
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.dropped_scans = scan_assembler_.discarded_scans();
      }
    }
  }

  void publish_imu(const ImuSample & sample)
  {
    sensor_msgs::msg::Imu message;
    message.header.stamp = now();  // Host reception time; no shared hardware clock.
    message.header.frame_id = imu_frame_;
    message.orientation_covariance[0] = -1.0;  // Raw sensor, no attitude estimate.
    message.linear_acceleration.x = sample.acceleration[0];
    message.linear_acceleration.y = sample.acceleration[1];
    message.linear_acceleration.z = sample.acceleration[2];
    message.angular_velocity.x = sample.angular_velocity[0];
    message.angular_velocity.y = sample.angular_velocity[1];
    message.angular_velocity.z = sample.angular_velocity[2];
    // All-zero raw covariances mean unknown until this chassis is calibrated.
    imu_publisher_->publish(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++state_.imu_samples;
    state_.last_imu_time = std::chrono::steady_clock::now();
  }

  void publish_scan(const CompleteScan & scan)
  {
    sensor_msgs::msg::LaserScan message;
    message.header.stamp = now();  // Complete-scan reception time, not synchronized acquisition.
    message.header.frame_id = laser_frame_;
    message.angle_min = 0.0F;
    message.angle_increment = static_cast<float>(2.0 * M_PI / scan_bins_);
    message.angle_max = (scan_bins_ - 1) * message.angle_increment;
    message.range_min = static_cast<float>(scan_range_min_);
    message.range_max = static_cast<float>(scan_range_max_);
    message.scan_time = 0.0F;
    if (have_scan_stamp_) {
      const double interval = static_cast<uint32_t>(scan.stamp_ms - last_scan_stamp_) * 0.001;
      if (interval >= 0.02 && interval <= 2.0) {
        message.scan_time = static_cast<float>(interval);
      }
    }
    last_scan_stamp_ = scan.stamp_ms;
    have_scan_stamp_ = true;
    message.time_increment = 0.0F;  // Reordered angular bins have no per-ray timing.
    message.ranges.assign(scan_bins_, std::numeric_limits<float>::infinity());
    for (const auto & point : scan.points) {
      const float range = point.range_mm * 0.001F;
      if (range < message.range_min || range > message.range_max) {
        continue;
      }
      double angle = std::fmod(
        scan_angle_offset_ +
        scan_angle_direction_ * point.angle_cdeg * 0.01, 360.0);
      if (angle < 0) {
        angle += 360.0;
      }
      const auto index = static_cast<std::size_t>(angle * scan_bins_ / 360.0);
      message.ranges[index] = std::min(message.ranges[index], range);
    }
    scan_publisher_->publish(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++state_.scans;
    state_.last_scan_time = std::chrono::steady_clock::now();
  }

  void process_status(const RobotStatus & status)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const rclcpp::Time stamp = now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_.status = status;
      state_.last_status_time = steady_now;
      state_.have_status = true;
    }

    publish_battery(status, stamp);
    std_msgs::msg::Int32MultiArray encoders;
    encoders.data = {status.left_encoder_count, status.right_encoder_count};
    encoder_publisher_->publish(encoders);

    OdometryUpdate update;
    if (!odometry_.update(
        status.left_encoder_count, status.right_encoder_count,
        steady_seconds(steady_now), update))
    {
      return;
    }
    publish_odometry(update, stamp);
  }

  void publish_odometry(const OdometryUpdate & update, const rclcpp::Time & stamp)
  {
    const double half_yaw = update.yaw * 0.5;
    const double qz = std::sin(half_yaw);
    const double qw = std::cos(half_yaw);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = update.x;
    odom.pose.pose.position.y = update.y;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom.twist.twist.linear.x = update.linear_velocity;
    odom.twist.twist.angular.z = update.angular_velocity;
    set_covariance_diagonal(odom.pose.covariance, pose_covariance_diagonal_);
    set_covariance_diagonal(odom.twist.covariance, twist_covariance_diagonal_);
    odom_publisher_->publish(odom);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = update.x;
      transform.transform.translation.y = update.y;
      transform.transform.rotation.z = qz;
      transform.transform.rotation.w = qw;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publish_battery(const RobotStatus & status, const rclcpp::Time & stamp)
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = stamp;
    battery.header.frame_id = battery_frame_;
    battery.voltage = status.battery_voltage;
    battery.temperature = nan;
    battery.current = nan;
    battery.charge = nan;
    battery.capacity = nan;
    battery.design_capacity = nan;
    battery.percentage = nan;
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    battery.power_supply_health = (status.status_flags & 0x04U) != 0U ?
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE :
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    battery.present = status.battery_voltage > 0.5F;
    battery.location = "smart_base";
    battery_publisher_->publish(battery);
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    DriverState state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state = state_;
    }

    const auto steady_now = std::chrono::steady_clock::now();
    const double status_age = state.have_status ?
      std::chrono::duration<double>(steady_now - state.last_status_time).count() :
      std::numeric_limits<double>::infinity();

    double command_age = std::numeric_limits<double>::infinity();
    bool have_command = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      have_command = have_command_;
      if (have_command_) {
        command_age = std::chrono::duration<double>(steady_now - last_command_time_).count();
      }
    }

    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "Communication OK";
    if (!state.connected) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "Serial port disconnected";
    } else if (!state.have_status) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "Waiting for ESP32 status";
    } else if (status_age > status_timeout_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "ESP32 status is stale";
    } else if ((state.status.status_flags & 0x02U) != 0U) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "ESP32 command timeout reported";
    } else if ((state.status.status_flags & 0x04U) != 0U) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "Low battery reported";
    }
    const double imu_age = state.last_imu_time.time_since_epoch().count() == 0 ?
      std::numeric_limits<double>::infinity() :
      std::chrono::duration<double>(steady_now - state.last_imu_time).count();
    const double scan_age = state.last_scan_time.time_since_epoch().count() == 0 ?
      std::numeric_limits<double>::infinity() :
      std::chrono::duration<double>(steady_now - state.last_scan_time).count();
    if (require_sensors_ && level == diagnostic_msgs::msg::DiagnosticStatus::OK &&
      (imu_age > 0.5 || scan_age > 1.0))
    {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "IMU or lidar data is stale";
    }
    diagnostic.summary(level, message);
    diagnostic.add("IMU age (s)", imu_age);
    diagnostic.add("Scan age (s)", scan_age);
    diagnostic.add("IMU samples", state.imu_samples);
    diagnostic.add("Complete scans", state.scans);
    diagnostic.add("Discarded scans", state.dropped_scans);
    diagnostic.add("Invalid sensor frames", state.invalid_sensors);

    diagnostic.add("Port", port_);
    diagnostic.add("Connected", state.connected);
    diagnostic.add("Status age (s)", status_age);
    diagnostic.add("Have cmd_vel", have_command);
    diagnostic.add("Command age (s)", command_age);
    diagnostic.add("Command timeout enabled", enable_cmd_vel_timeout_);
    diagnostic.add("Transmitted frames", state.tx_frames);
    diagnostic.add("Connection count", state.reconnects);
    diagnostic.add("Parser bad checksum", state.parser_bad_checksum);
    diagnostic.add("Parser bad length", state.parser_bad_length);
    diagnostic.add("Invalid status frames", state.invalid_status);
    diagnostic.add("Protocol version", static_cast<int>(state.status.protocol_version));
    diagnostic.add("Firmware version", state.status.firmware_version);
    diagnostic.add("Battery voltage", state.status.battery_voltage);
    diagnostic.add("Left RPM", state.status.left_rpm * left_encoder_sign_);
    diagnostic.add("Right RPM", state.status.right_rpm * right_encoder_sign_);
    diagnostic.add("Left encoder", state.status.left_encoder_count);
    diagnostic.add("Right encoder", state.status.right_encoder_count);
    diagnostic.add("ESP32 received bytes", state.status.rx_byte_count);
    diagnostic.add("ESP32 valid commands", state.status.rx_valid_cmd_count);
    diagnostic.add("ESP32 bad checksum", state.status.rx_bad_checksum_count);
    diagnostic.add("ESP32 bad length", state.status.rx_bad_len_count);
  }

  const std::string port_;
  const int baud_rate_;
  const double wheel_diameter_;
  const double wheel_base_;
  const int64_t counts_per_wheel_rev_;
  const int left_encoder_sign_;
  const int right_encoder_sign_;
  const double command_rate_;
  const double cmd_vel_timeout_;
  const bool enable_cmd_vel_timeout_;
  const double status_timeout_;
  const double reconnect_interval_;
  const double max_linear_velocity_;
  const double max_angular_velocity_;
  const int32_t max_encoder_delta_;
  const bool publish_tf_;
  const std::string odom_frame_;
  const std::string base_frame_;
  const std::string battery_frame_;
  const std::string imu_frame_;
  const std::string laser_frame_;
  const int scan_bins_;
  const double scan_range_min_;
  const double scan_range_max_;
  const double scan_angle_offset_;
  const double scan_angle_direction_;
  const bool require_sensors_;
  const std::vector<double> pose_covariance_diagonal_;
  const std::vector<double> twist_covariance_diagonal_;

  DifferentialOdometry odometry_;
  FrameParser parser_;
  ScanAssembler scan_assembler_;
  uint32_t last_scan_stamp_{0};
  bool have_scan_stamp_{false};
  diagnostic_updater::Updater updater_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::atomic<bool> running_{false};
  std::thread io_thread_;
  int serial_fd_{-1};

  std::mutex command_mutex_;
  double command_linear_{0.0};
  double command_angular_{0.0};
  bool have_command_{false};
  std::chrono::steady_clock::time_point last_command_time_{};
  uint64_t command_sequence_{0};

  // Used only by the serial I/O thread, independently of ROS callback timing.
  uint64_t last_sent_command_sequence_{0};
  std::chrono::steady_clock::time_point last_command_sent_time_{};

  std::mutex state_mutex_;
  DriverState state_;
};

}  // namespace smart_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<smart_bringup::SmartBaseNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("smart_base_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
