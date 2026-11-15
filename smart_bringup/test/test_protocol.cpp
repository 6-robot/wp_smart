#include <climits>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "gtest/gtest.h"

#include "smart_bringup/protocol.hpp"

namespace
{

void append_u16(std::vector<uint8_t> & data, uint16_t value)
{
  data.push_back(static_cast<uint8_t>(value));
  data.push_back(static_cast<uint8_t>(value >> 8U));
}

void append_u32(std::vector<uint8_t> & data, uint32_t value)
{
  data.push_back(static_cast<uint8_t>(value));
  data.push_back(static_cast<uint8_t>(value >> 8U));
  data.push_back(static_cast<uint8_t>(value >> 16U));
  data.push_back(static_cast<uint8_t>(value >> 24U));
}

void append_float(std::vector<uint8_t> & data, float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u32(data, bits);
}

std::vector<uint8_t> make_status_payload()
{
  std::vector<uint8_t> payload;
  append_float(payload, 10.0F);
  append_float(payload, -11.0F);
  append_float(payload, 12.0F);
  append_float(payload, -13.0F);
  append_u32(payload, 1000U);
  append_u32(payload, static_cast<uint32_t>(-2000));
  append_float(payload, 11.7F);
  append_u16(payload, static_cast<uint16_t>(120));
  append_u16(payload, static_cast<uint16_t>(-121));
  payload.push_back(0x01U);
  append_u32(payload, 2026061505U);
  append_u32(payload, 10000U);
  append_u32(payload, 500U);
  append_u32(payload, 2U);
  append_u32(payload, 3U);
  return payload;
}

}  // namespace

TEST(Protocol, CommandRoundTripWithSplitInput)
{
  const auto command = smart_bringup::make_cmd_vel(0.25F, -0.5F);
  smart_bringup::FrameParser parser;
  const auto first = parser.feed(command.data(), 3);
  EXPECT_TRUE(first.empty());
  const auto second = parser.feed(command.data() + 3, command.size() - 3);
  ASSERT_EQ(second.size(), 1U);
  EXPECT_EQ(second[0].type, smart_bringup::kMsgCmdVel);
  ASSERT_EQ(second[0].payload.size(), 8U);
}

TEST(Protocol, ParserRecoversFromGarbageAndBadChecksum)
{
  auto bad = smart_bringup::make_frame(smart_bringup::kMsgStatus, make_status_payload());
  bad.back() ^= 0xFFU;
  const auto good = smart_bringup::make_frame(smart_bringup::kMsgStatus, make_status_payload());
  std::vector<uint8_t> stream = {0x00, 0xAA, 0x01, 0x55};
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), good.begin(), good.end());

  smart_bringup::FrameParser parser;
  const auto frames = parser.feed(stream.data(), stream.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].type, smart_bringup::kMsgStatus);
  EXPECT_EQ(parser.bad_checksum_count(), 1U);
}

TEST(Protocol, DecodesVersionThreeStatus)
{
  smart_bringup::RobotStatus status;
  ASSERT_TRUE(smart_bringup::decode_status(make_status_payload(), status));
  EXPECT_EQ(status.protocol_version, 3U);
  EXPECT_FLOAT_EQ(status.left_rpm, 10.0F);
  EXPECT_FLOAT_EQ(status.right_rpm, -11.0F);
  EXPECT_EQ(status.left_encoder_count, 1000);
  EXPECT_EQ(status.right_encoder_count, -2000);
  EXPECT_FLOAT_EQ(status.battery_voltage, 11.7F);
  EXPECT_EQ(status.firmware_version, 2026061505U);
  EXPECT_EQ(status.rx_valid_cmd_count, 500U);
}

TEST(Protocol, HandlesSignedEncoderWrap)
{
  EXPECT_EQ(smart_bringup::wrapped_encoder_delta(INT32_MIN + 4, INT32_MAX - 5), 10);
  EXPECT_EQ(smart_bringup::wrapped_encoder_delta(INT32_MAX - 4, INT32_MIN + 5), -10);
}

TEST(Odometry, AppliesEncoderSignsAndIntegratesStraightMotion)
{
  const double wheel_diameter = 1.0 / M_PI;
  smart_bringup::DifferentialOdometry odometry(wheel_diameter, 1.0, 1000, 1, -1, 10000);
  smart_bringup::OdometryUpdate update;
  EXPECT_FALSE(odometry.update(0, 0, 1.0, update));
  ASSERT_TRUE(odometry.update(100, -100, 2.0, update));
  EXPECT_NEAR(update.x, 0.1, 1.0e-9);
  EXPECT_NEAR(update.y, 0.0, 1.0e-9);
  EXPECT_NEAR(update.yaw, 0.0, 1.0e-9);
  EXPECT_NEAR(update.linear_velocity, 0.1, 1.0e-9);
}

TEST(Odometry, RejectsImplausibleCountJumpAndResetsBaseline)
{
  smart_bringup::DifferentialOdometry odometry(0.1, 0.2, 1000, 1, -1, 100);
  smart_bringup::OdometryUpdate update;
  EXPECT_FALSE(odometry.update(0, 0, 1.0, update));
  EXPECT_FALSE(odometry.update(1000, -1000, 2.0, update));
  ASSERT_TRUE(odometry.update(1010, -1010, 3.0, update));
  EXPECT_EQ(update.left_delta, 10);
  EXPECT_EQ(update.right_delta, 10);
}

TEST(Odometry, IntegratesOneWheelRevolutionAt64ToOne)
{
  constexpr int64_t counts_per_wheel_rev = 1024 * 4 * 64;
  constexpr double wheel_diameter = 0.0566;
  const double circumference = M_PI * wheel_diameter;
  smart_bringup::DifferentialOdometry odometry(
    wheel_diameter, 0.195, counts_per_wheel_rev, 1, -1, 256000);
  smart_bringup::OdometryUpdate update;
  EXPECT_FALSE(odometry.update(0, 0, 1.0, update));
  for (int quarter = 1; quarter <= 4; ++quarter) {
    const int32_t count = static_cast<int32_t>(counts_per_wheel_rev / 4 * quarter);
    ASSERT_TRUE(odometry.update(count, -count, 1.0 + quarter * 0.25, update));
    EXPECT_NEAR(update.x, circumference * quarter / 4.0, 1.0e-9);
    EXPECT_NEAR(update.linear_velocity, circumference, 1.0e-9);
    EXPECT_NEAR(update.yaw, 0.0, 1.0e-9);
  }
}

TEST(Odometry, ScalesCountJumpLimitFor64ToOne)
{
  smart_bringup::DifferentialOdometry odometry(0.0566, 0.195, 262144, 1, -1, 256000);
  smart_bringup::OdometryUpdate update;
  EXPECT_FALSE(odometry.update(0, 0, 1.0, update));
  ASSERT_TRUE(odometry.update(250000, -250000, 2.0, update));
  EXPECT_NEAR(update.x, 250000.0 * M_PI * 0.0566 / 262144.0, 1.0e-9);
  EXPECT_FALSE(odometry.update(506001, -506001, 3.0, update));
}

TEST(Sensors, ImuRejectsLengthAndNonFinite)
{
  std::vector<uint8_t> payload;
  append_u32(payload, 1234);
  for (float value : {1.0F, 2.0F, 9.80665F, 0.1F, 0.2F, 0.3F}) {
    append_float(payload, value);
  }
  smart_bringup::ImuSample sample;
  ASSERT_TRUE(smart_bringup::decode_imu(payload, sample));
  EXPECT_EQ(sample.stamp_ms, 1234U);
  EXPECT_FLOAT_EQ(sample.acceleration[2], 9.80665F);
  EXPECT_FLOAT_EQ(sample.angular_velocity[1], 0.2F);
  auto invalid = payload;
  invalid.pop_back();
  EXPECT_FALSE(smart_bringup::decode_imu(invalid, sample));
  invalid = payload;
  invalid[26] = 0xC0;
  invalid[27] = 0x7F;  // NaN in the final gyro value.
  EXPECT_FALSE(smart_bringup::decode_imu(invalid, sample));
}

TEST(Sensors, ScanFragmentBoundsAndCompletion)
{
  std::vector<uint8_t> payload;
  append_u32(payload, 500);
  append_u16(payload, 65535);
  append_u16(payload, 2);
  append_u16(payload, 0);
  payload.push_back(1);
  payload.push_back(0);
  append_u16(payload, 9000);
  append_u16(payload, 1000);
  smart_bringup::ScanFragment fragment;
  ASSERT_TRUE(smart_bringup::decode_scan_fragment(payload, fragment));
  smart_bringup::ScanAssembler assembler;
  smart_bringup::CompleteScan scan;
  EXPECT_FALSE(assembler.add(fragment, scan));
  auto invalid = payload;
  invalid[11] = 1;  // Last flag must match offset + count == total.
  EXPECT_FALSE(smart_bringup::decode_scan_fragment(invalid, fragment));
  invalid = payload;
  invalid[12] = 0xA0;
  invalid[13] = 0x8C;  // 36000 cdeg is outside the wire angle interval.
  EXPECT_FALSE(smart_bringup::decode_scan_fragment(invalid, fragment));
  invalid = payload;
  invalid[6] = 0xFF;
  invalid[7] = 0xFF;
  EXPECT_FALSE(smart_bringup::decode_scan_fragment(invalid, fragment));
  payload[8] = 1;
  payload[11] = 1;
  ASSERT_TRUE(smart_bringup::decode_scan_fragment(payload, fragment));
  ASSERT_TRUE(assembler.add(fragment, scan));
  EXPECT_EQ(scan.scan_id, 65535U);
  ASSERT_EQ(scan.points.size(), 2U);
  EXPECT_EQ(scan.points[1].range_mm, 1000U);
  // Repeated/orphaned end fragments never publish an incomplete scan.
  EXPECT_FALSE(assembler.add(fragment, scan));
  payload[4] = 0;
  payload[5] = 0;  // scan id wrap is valid, not an ordering comparison.
  payload[8] = 0;
  payload[11] = 0;
  ASSERT_TRUE(smart_bringup::decode_scan_fragment(payload, fragment));
  EXPECT_FALSE(assembler.add(fragment, scan));
  payload[8] = 1;
  payload[11] = 1;
  payload[4] = 1;  // Missing/mismatched scan, discard the partial buffer.
  ASSERT_TRUE(smart_bringup::decode_scan_fragment(payload, fragment));
  EXPECT_FALSE(assembler.add(fragment, scan));
  EXPECT_EQ(assembler.discarded_scans(), 1U);
}
