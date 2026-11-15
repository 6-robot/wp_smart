#include "smart_bringup/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace smart_bringup
{
namespace
{

uint16_t read_u16_le(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t read_u32_le(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) |
         (static_cast<uint32_t>(data[3]) << 24U);
}

float read_float_le(const uint8_t * data)
{
  const uint32_t bits = read_u32_le(data);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void append_u32_le(std::vector<uint8_t> & output, uint32_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xFFU));
  output.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
  output.push_back(static_cast<uint8_t>((value >> 16U) & 0xFFU));
  output.push_back(static_cast<uint8_t>((value >> 24U) & 0xFFU));
}

void append_float_le(std::vector<uint8_t> & output, float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u32_le(output, bits);
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

uint8_t checksum(uint8_t type, const std::vector<uint8_t> & payload)
{
  uint8_t sum = static_cast<uint8_t>(type + static_cast<uint8_t>(payload.size()));
  for (const uint8_t byte : payload) {
    sum = static_cast<uint8_t>(sum + byte);
  }
  return sum;
}

std::vector<uint8_t> make_frame(uint8_t type, const std::vector<uint8_t> & payload)
{
  if (payload.size() > kMaxPayload) {
    throw std::invalid_argument("payload exceeds protocol limit");
  }

  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 5U);
  frame.push_back(kFrameHead1);
  frame.push_back(kFrameHead2);
  frame.push_back(type);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(checksum(type, payload));
  return frame;
}

std::vector<uint8_t> make_cmd_vel(float linear_x, float angular_z)
{
  std::vector<uint8_t> payload;
  payload.reserve(8);
  append_float_le(payload, linear_x);
  append_float_le(payload, angular_z);
  return make_frame(kMsgCmdVel, payload);
}

bool decode_status(const std::vector<uint8_t> & payload, RobotStatus & status)
{
  if (payload.size() != 33U && payload.size() != 37U && payload.size() != 53U) {
    return false;
  }

  status = RobotStatus{};
  status.left_rpm = read_float_le(payload.data());
  status.right_rpm = read_float_le(payload.data() + 4);
  status.left_target_rpm = read_float_le(payload.data() + 8);
  status.right_target_rpm = read_float_le(payload.data() + 12);
  status.left_encoder_count = static_cast<int32_t>(read_u32_le(payload.data() + 16));
  status.right_encoder_count = static_cast<int32_t>(read_u32_le(payload.data() + 20));
  status.battery_voltage = read_float_le(payload.data() + 24);
  status.left_pwm = static_cast<int16_t>(read_u16_le(payload.data() + 28));
  status.right_pwm = static_cast<int16_t>(read_u16_le(payload.data() + 30));
  status.status_flags = payload[32];
  status.protocol_version = 1;

  if (payload.size() >= 37U) {
    status.firmware_version = read_u32_le(payload.data() + 33);
    status.protocol_version = 2;
  }
  if (payload.size() == 53U) {
    status.rx_byte_count = read_u32_le(payload.data() + 37);
    status.rx_valid_cmd_count = read_u32_le(payload.data() + 41);
    status.rx_bad_checksum_count = read_u32_le(payload.data() + 45);
    status.rx_bad_len_count = read_u32_le(payload.data() + 49);
    status.protocol_version = 3;
  }

  return std::isfinite(status.left_rpm) && std::isfinite(status.right_rpm) &&
         std::isfinite(status.left_target_rpm) && std::isfinite(status.right_target_rpm) &&
         std::isfinite(status.battery_voltage);
}

int32_t wrapped_encoder_delta(int32_t current, int32_t previous)
{
  const uint32_t unsigned_delta = static_cast<uint32_t>(current) -
    static_cast<uint32_t>(previous);
  int32_t signed_delta = 0;
  std::memcpy(&signed_delta, &unsigned_delta, sizeof(signed_delta));
  return signed_delta;
}

std::vector<Frame> FrameParser::feed(const uint8_t * data, std::size_t size)
{
  if (data != nullptr && size > 0U) {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  std::vector<Frame> frames;
  while (true) {
    const uint8_t header[] = {kFrameHead1, kFrameHead2};
    const auto start = std::search(
      buffer_.begin(), buffer_.end(), std::begin(header), std::end(header));
    if (start == buffer_.end()) {
      const bool keep_head = !buffer_.empty() && buffer_.back() == kFrameHead1;
      buffer_.clear();
      if (keep_head) {
        buffer_.push_back(kFrameHead1);
      }
      break;
    }
    buffer_.erase(buffer_.begin(), start);

    if (buffer_.size() < 5U) {
      break;
    }

    const uint8_t type = buffer_[2];
    const std::size_t payload_size = buffer_[3];
    if (payload_size > kMaxPayload) {
      ++bad_length_count_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t frame_size = payload_size + 5U;
    if (buffer_.size() < frame_size) {
      break;
    }

    std::vector<uint8_t> payload(
      buffer_.begin() + 4, buffer_.begin() + 4 + static_cast<std::ptrdiff_t>(payload_size));
    if (buffer_[frame_size - 1U] != checksum(type, payload)) {
      ++bad_checksum_count_;
      buffer_.erase(buffer_.begin());
      continue;
    }

    frames.push_back(Frame{type, std::move(payload)});
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
  }
  return frames;
}

void FrameParser::reset()
{
  buffer_.clear();
}

bool decode_imu(const std::vector<uint8_t> & payload, ImuSample & sample)
{
  if (payload.size() != 28U) {
    return false;
  }
  ImuSample result;
  result.stamp_ms = read_u32_le(payload.data());
  for (std::size_t i = 0; i < 3; ++i) {
    result.acceleration[i] = read_float_le(payload.data() + 4 + i * 4);
    result.angular_velocity[i] = read_float_le(payload.data() + 16 + i * 4);
    if (!std::isfinite(result.acceleration[i]) || !std::isfinite(result.angular_velocity[i])) {
      return false;
    }
  }
  sample = result;
  return true;
}

bool decode_scan_fragment(const std::vector<uint8_t> & payload, ScanFragment & fragment)
{
  if (payload.size() < 16U) {
    return false;
  }
  ScanFragment result;
  result.stamp_ms = read_u32_le(payload.data());
  result.scan_id = read_u16_le(payload.data() + 4);
  result.total_points = read_u16_le(payload.data() + 6);
  result.offset = read_u16_le(payload.data() + 8);
  const std::size_t count = payload[10];
  const uint8_t flags = payload[11];
  result.last = (flags & 1U) != 0;
  if (result.total_points == 0 || result.total_points > kMaxScanPoints ||
    count == 0 || count > 12 || payload.size() != 12U + count * 4U ||
    result.offset + count > result.total_points || flags > 1 ||
    result.last != (result.offset + count == result.total_points))
  {
    return false;
  }
  result.points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    ScanPoint point{read_u16_le(payload.data() + 12 + i * 4),
      read_u16_le(payload.data() + 14 + i * 4)};
    if (point.angle_cdeg >= 36000) {
      return false;
    }
    result.points.push_back(point);
  }
  fragment = std::move(result);
  return true;
}

bool ScanAssembler::add(const ScanFragment & fragment, CompleteScan & scan)
{
  if (fragment.offset == 0) {
    if (active_) {
      ++discarded_scans_;
    }
    current_ = CompleteScan{fragment.stamp_ms, fragment.scan_id, {}};
    expected_points_ = fragment.total_points;
    active_ = true;
  }
  if (!active_) {
    return false;
  }
  if (fragment.scan_id != current_.scan_id || fragment.stamp_ms != current_.stamp_ms ||
    fragment.total_points != expected_points_ || fragment.offset != current_.points.size() ||
    current_.points.size() + fragment.points.size() > expected_points_)
  {
    ++discarded_scans_;
    reset();
    return false;
  }
  current_.points.insert(current_.points.end(), fragment.points.begin(), fragment.points.end());
  if (!fragment.last || current_.points.size() != expected_points_) {
    return false;
  }
  scan = std::move(current_);
  active_ = false;
  return true;
}

void ScanAssembler::reset()
{
  active_ = false;
  expected_points_ = 0;
  current_ = CompleteScan{};
}

DifferentialOdometry::DifferentialOdometry(
  double wheel_diameter, double wheel_base, int64_t counts_per_wheel_rev,
  int left_sign, int right_sign, int32_t max_encoder_delta)
: wheel_base_(wheel_base),
  meters_per_count_(M_PI * wheel_diameter / static_cast<double>(counts_per_wheel_rev)),
  left_sign_(left_sign),
  right_sign_(right_sign),
  max_encoder_delta_(max_encoder_delta)
{
  if (wheel_diameter <= 0.0 || wheel_base <= 0.0 || counts_per_wheel_rev <= 0) {
    throw std::invalid_argument("invalid differential drive geometry");
  }
  if ((left_sign != 1 && left_sign != -1) || (right_sign != 1 && right_sign != -1)) {
    throw std::invalid_argument("encoder signs must be 1 or -1");
  }
  if (max_encoder_delta <= 0) {
    throw std::invalid_argument("max_encoder_delta must be positive");
  }
}

bool DifferentialOdometry::update(
  int32_t left_count, int32_t right_count, double timestamp_seconds,
  OdometryUpdate & result)
{
  if (!initialized_) {
    previous_left_ = left_count;
    previous_right_ = right_count;
    previous_time_ = timestamp_seconds;
    initialized_ = true;
    return false;
  }

  const int32_t raw_left_delta = wrapped_encoder_delta(left_count, previous_left_);
  const int32_t raw_right_delta = wrapped_encoder_delta(right_count, previous_right_);
  const double dt = timestamp_seconds - previous_time_;
  previous_left_ = left_count;
  previous_right_ = right_count;
  previous_time_ = timestamp_seconds;

  if (dt <= 0.0 || std::abs(static_cast<int64_t>(raw_left_delta)) > max_encoder_delta_ ||
    std::abs(static_cast<int64_t>(raw_right_delta)) > max_encoder_delta_)
  {
    return false;
  }

  const int32_t left_delta = raw_left_delta * left_sign_;
  const int32_t right_delta = raw_right_delta * right_sign_;
  const double left_distance = static_cast<double>(left_delta) * meters_per_count_;
  const double right_distance = static_cast<double>(right_delta) * meters_per_count_;
  const double center_distance = (left_distance + right_distance) * 0.5;
  const double yaw_delta = (right_distance - left_distance) / wheel_base_;

  x_ += center_distance * std::cos(yaw_ + yaw_delta * 0.5);
  y_ += center_distance * std::sin(yaw_ + yaw_delta * 0.5);
  yaw_ = normalize_angle(yaw_ + yaw_delta);

  result.x = x_;
  result.y = y_;
  result.yaw = yaw_;
  result.linear_velocity = center_distance / dt;
  result.angular_velocity = yaw_delta / dt;
  result.left_delta = left_delta;
  result.right_delta = right_delta;
  return true;
}

void DifferentialOdometry::reset_baseline()
{
  initialized_ = false;
}

}  // namespace smart_bringup
