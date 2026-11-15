#ifndef SMART_BRINGUP__PROTOCOL_HPP_
#define SMART_BRINGUP__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace smart_bringup
{

constexpr uint8_t kFrameHead1 = 0xAA;
constexpr uint8_t kFrameHead2 = 0x55;
constexpr uint8_t kMsgCmdVel = 0x01;
constexpr uint8_t kMsgStatus = 0x81;
constexpr uint8_t kMsgEstop = 0x02;
constexpr uint8_t kMsgImu = 0x82;
constexpr uint8_t kMsgScanFragment = 0x83;
constexpr std::size_t kMaxPayload = 64;
constexpr std::size_t kMaxScanPoints = 1200;

struct Frame
{
  uint8_t type{0};
  std::vector<uint8_t> payload;
};

struct RobotStatus
{
  float left_rpm{0.0F};
  float right_rpm{0.0F};
  float left_target_rpm{0.0F};
  float right_target_rpm{0.0F};
  int32_t left_encoder_count{0};
  int32_t right_encoder_count{0};
  float battery_voltage{0.0F};
  int16_t left_pwm{0};
  int16_t right_pwm{0};
  uint8_t status_flags{0};
  uint32_t firmware_version{0};
  uint32_t rx_byte_count{0};
  uint32_t rx_valid_cmd_count{0};
  uint32_t rx_bad_checksum_count{0};
  uint32_t rx_bad_len_count{0};
  uint8_t protocol_version{0};
};

uint8_t checksum(uint8_t type, const std::vector<uint8_t> & payload);
std::vector<uint8_t> make_frame(uint8_t type, const std::vector<uint8_t> & payload);
std::vector<uint8_t> make_cmd_vel(float linear_x, float angular_z);
bool decode_status(const std::vector<uint8_t> & payload, RobotStatus & status);
int32_t wrapped_encoder_delta(int32_t current, int32_t previous);

struct ImuSample
{
  uint32_t stamp_ms{0};
  std::array<float, 3> acceleration{};
  std::array<float, 3> angular_velocity{};
};

struct ScanPoint
{
  uint16_t angle_cdeg{0};
  uint16_t range_mm{0};
};

struct ScanFragment
{
  uint32_t stamp_ms{0};
  uint16_t scan_id{0};
  uint16_t total_points{0};
  uint16_t offset{0};
  bool last{false};
  std::vector<ScanPoint> points;
};

struct CompleteScan
{
  uint32_t stamp_ms{0};
  uint16_t scan_id{0};
  std::vector<ScanPoint> points;
};

bool decode_imu(const std::vector<uint8_t> & payload, ImuSample & sample);
bool decode_scan_fragment(const std::vector<uint8_t> & payload, ScanFragment & fragment);

class ScanAssembler
{
public:
  bool add(const ScanFragment & fragment, CompleteScan & scan);
  void reset();
  uint64_t discarded_scans() const {return discarded_scans_;}

private:
  bool active_{false};
  uint16_t expected_points_{0};
  CompleteScan current_;
  uint64_t discarded_scans_{0};
};

class FrameParser
{
public:
  std::vector<Frame> feed(const uint8_t * data, std::size_t size);
  void reset();

  uint64_t bad_checksum_count() const {return bad_checksum_count_;}
  uint64_t bad_length_count() const {return bad_length_count_;}

private:
  std::vector<uint8_t> buffer_;
  uint64_t bad_checksum_count_{0};
  uint64_t bad_length_count_{0};
};

struct OdometryUpdate
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  int32_t left_delta{0};
  int32_t right_delta{0};
};

class DifferentialOdometry
{
public:
  DifferentialOdometry(
    double wheel_diameter, double wheel_base, int64_t counts_per_wheel_rev,
    int left_sign, int right_sign, int32_t max_encoder_delta);

  bool update(
    int32_t left_count, int32_t right_count, double timestamp_seconds,
    OdometryUpdate & result);
  void reset_baseline();

private:
  double wheel_base_;
  double meters_per_count_;
  int left_sign_;
  int right_sign_;
  int32_t max_encoder_delta_;
  bool initialized_{false};
  int32_t previous_left_{0};
  int32_t previous_right_{0};
  double previous_time_{0.0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
};

}  // namespace smart_bringup

#endif  // SMART_BRINGUP__PROTOCOL_HPP_
