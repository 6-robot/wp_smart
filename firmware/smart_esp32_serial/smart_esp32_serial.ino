// ESP32-S3 serial chassis: Arduino-ESP32 2.0.17, no Wi-Fi transport.
// GPIO17 TX / GPIO18 RX -> WCH host adapter, 460800 8N1.
#include <Arduino.h>
#include <Wire.h>
#include <Ticker.h>
#include "driver/pcnt.h"
#include "motion_guard.h"
#include "display_data.h"
#include "status_display.h"

#define ENC1_A_PIN 4
#define ENC1_B_PIN 5
#define ENC2_A_PIN 6
#define ENC2_B_PIN 7
#define ENCODER_PPR 1024
#define COUNTS_PER_REV (ENCODER_PPR * 4)
#define REDUCTION_RATIO 64
#define WHEEL_DIAMETER_M 0.0657f
#define WHEEL_BASE_M 0.1805f
#define MAX_LINEAR_MPS 0.5f
#define MAX_ANGULAR_RADPS 2.5f
#define CMD_TIMEOUT_MS 3000
#define FIRMWARE_VERSION 2026111506UL
#define MOTOR1_IN1 11
#define MOTOR1_IN2 12
#define MOTOR2_IN1 13
#define MOTOR2_IN2 14
#define LEDC_M1_IN1 0
#define LEDC_M1_IN2 1
#define LEDC_M2_IN1 2
#define LEDC_M2_IN2 3
#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_RES 8
#define LIDAR_MAX_SAMPLES 80
#define LIDAR_POLL_BYTES 1024
#define MAX_SCAN_POINTS 1200
#define DISPLAY_UPDATE_MS 500

HardwareSerial HostSerial(1);
HardwareSerial LidarSerial(2);
Ticker pidTicker;
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;
MotionGuard motion;
struct MotorState {
  float rpm[2] = {};
  float targets[2] = {};
  int32_t encoders[2] = {};
  int16_t pwm[2] = {};
};
MotorState motorState;
uint32_t rxByteCount = 0, rxValidCmdCount = 0;
uint32_t rxBadChecksumCount = 0, rxBadLenCount = 0;
uint32_t lidarBadChecksumCount = 0, lidarPacketCount = 0;
uint32_t lidarLastPacketMs = 0, lidarDroppedScans = 0;
float batteryVoltage = 0;
bool imuReady = false;
uint8_t imuAddress = 0;
uint32_t imuLastSampleMs = 0, imuLastRetryMs = 0;
uint8_t imuPayload[28];
bool imuPending = false;
struct ScanPoint { uint16_t angle_cdeg, range_mm; };
ScanPoint scans[2][MAX_SCAN_POINTS];
uint8_t writeIndex = 0, sendIndex = 1;
uint16_t writeCount = 0, sendCount = 0, sendOffset = 0;
uint16_t nextScanId = 0, sendScanId = 0;
uint32_t writeStamp = 0, sendStamp = 0;
bool scanAligned = false, scanOverflow = false;
FrontDistance frontDistance;
StatusDisplay statusDisplay;

void pcnt_encoder_init();
void motor_init();
void handleLidarPacket(uint8_t, uint8_t, uint16_t, uint16_t, uint16_t, const uint16_t*);

// Called by the independent 100 Hz task. loop() never writes PWM or PI state.
void pid_update() {
  float targets[2];
  portENTER_CRITICAL(&commandMux);
  motion.expire(millis(), CMD_TIMEOUT_MS);
  targets[0] = motion.left;
  targets[1] = motion.right;
  portEXIT_CRITICAL(&commandMux);
  static int16_t previous[2] = {};
  static uint32_t counts[2] = {};
  static float speeds[2] = {}, integrals[2] = {}, previousOutput[2] = {};
  MotorState next;
  const uint8_t channels[2][2] = {{0, 1}, {2, 3}};
  for (int i = 0; i < 2; ++i) {
    int16_t count = 0;
    pcnt_get_counter_value(i == 0 ? PCNT_UNIT_0 : PCNT_UNIT_1, &count);
    int32_t delta = int32_t(count) - previous[i];
    if (delta < -15000) delta += 30000;
    else if (delta > 15000) delta -= 30000;
    previous[i] = count;
    counts[i] += uint32_t(delta);  // Defined modulo-2^32 encoder accumulation.
    memcpy(&next.encoders[i], &counts[i], 4);
    float rpm = delta * 6000.0f / (REDUCTION_RATIO * COUNTS_PER_REV);
    speeds[i] = speeds[i] * 0.4f + rpm * 0.6f;
    int16_t pwm = 0;
    if (targets[i] == 0) {
      integrals[i] = 0;
    } else {
      float error = targets[i] - speeds[i];
      if ((previousOutput[i] > -255 && previousOutput[i] < 255) ||
          (previousOutput[i] >= 255 && error < 0) ||
          (previousOutput[i] <= -255 && error > 0)) {
        integrals[i] = constrain(integrals[i] + error * 0.01f, -80.0f/12, 80.0f/12);
      }
      float feedforward = 107.0f + fabsf(targets[i]) * 0.5f;
      if (fabsf(speeds[i]) < 2) feedforward += 20;
      feedforward = constrain(feedforward, 107.0f, 255.0f);
      if (targets[i] < 0) feedforward = -feedforward;
      pwm = int16_t(constrain(feedforward + error + 12 * integrals[i], -255.0f, 255.0f));
    }
    previousOutput[i] = pwm;
    if (pwm > 0) { ledcWrite(channels[i][0], pwm); ledcWrite(channels[i][1], 0); }
    else if (pwm < 0) { ledcWrite(channels[i][0], 0); ledcWrite(channels[i][1], -pwm); }
    else { ledcWrite(channels[i][0], 255); ledcWrite(channels[i][1], 255); }
    next.rpm[i] = speeds[i]; next.targets[i] = targets[i]; next.pwm[i] = pwm;
  }
  portENTER_CRITICAL(&telemetryMux);
  motorState = next;
  portEXIT_CRITICAL(&telemetryMux);
}

// Check the entire frame fits before writing: sensor traffic cannot block commands.
bool sendFrame(uint8_t type, const uint8_t* payload, uint8_t length) {
  if (HostSerial.availableForWrite() < length + 5) return false;
  uint8_t frame[69] = {0xAA, 0x55, type, length};
  uint8_t sum = type + length;
  for (uint8_t i = 0; i < length; ++i) { frame[4+i] = payload[i]; sum += payload[i]; }
  frame[4+length] = sum;
  return HostSerial.write(frame, length+5) == size_t(length+5);
}

void handleCommand(uint8_t type, const uint8_t* payload, uint8_t length) {
  if (type == 0x02 && length == 0) {
    portENTER_CRITICAL(&commandMux);
    motion.stop();
    portEXIT_CRITICAL(&commandMux);
    ++rxValidCmdCount;
  } else if (type == 0x01 && length == 8) {
    float linear, angular;
    memcpy(&linear, payload, 4); memcpy(&angular, payload+4, 4);
    if (!isfinite(linear) || !isfinite(angular)) { ++rxBadLenCount; return; }
    linear = constrain(linear, -MAX_LINEAR_MPS, MAX_LINEAR_MPS);
    angular = constrain(angular, -MAX_ANGULAR_RADPS, MAX_ANGULAR_RADPS);
    float scale = 60.0f / (PI * WHEEL_DIAMETER_M);
    float left = (linear - angular * WHEEL_BASE_M * 0.5f) * scale;
    float right = -(linear + angular * WHEEL_BASE_M * 0.5f) * scale;
    portENTER_CRITICAL(&commandMux);
    motion.accept(left, right, millis());
    portEXIT_CRITICAL(&commandMux);
    ++rxValidCmdCount;
  } else { ++rxBadLenCount; }
}

void pollHost() {
  static uint8_t buffer[69];
  static uint8_t size = 0;
  static uint32_t lastByteMs = 0;
  if (size && uint32_t(millis()-lastByteMs) > 50) size = 0;
  for (int budget = 0; budget < 512 && HostSerial.available(); ++budget) {
    uint8_t byte = HostSerial.read();
    ++rxByteCount; lastByteMs = millis();
    buffer[size++] = byte;
    while (size) {
      if (buffer[0] != 0xAA || (size >= 2 && buffer[1] != 0x55)) {
        memmove(buffer, buffer+1, --size); continue;
      }
      if (size < 4) break;
      uint8_t len = buffer[3];
      if (len > 64) { ++rxBadLenCount; memmove(buffer, buffer+1, --size); continue; }
      if (size < len+5) break;
      uint8_t sum = buffer[2] + len;
      for (uint8_t i = 0; i < len; ++i) sum += buffer[4+i];
      if (sum == buffer[4+len]) {
        handleCommand(buffer[2], buffer+4, len);
        // A valid frame found while resynchronizing may have queued bytes behind it.
        uint8_t consumed = len+5;
        size -= consumed;
        memmove(buffer, buffer+consumed, size);
      }
      else { ++rxBadChecksumCount; memmove(buffer, buffer+1, --size); }
    }
  }
}

bool imuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
bool imuRead(uint8_t reg, uint8_t* data, uint8_t size) {
  Wire.beginTransmission(imuAddress); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imuAddress, size, uint8_t(true)) != size) return false;
  for (uint8_t i = 0; i < size; ++i) data[i] = Wire.read();
  return true;
}
void initImu() {
  imuReady = false;
  for (uint8_t address : {uint8_t(0x68), uint8_t(0x69)}) {
    imuAddress = address;
    uint8_t id = 0;
    if (!imuRead(0x75, &id, 1) || id != 0x68) continue;
    // MPU-60x0: PLL X gyro clock, DLPF 44/42 Hz, 100 Hz, +/-250 dps, +/-2g.
    imuReady = imuWrite(0x6B, 1) && imuWrite(0x6C, 0) && imuWrite(0x1A, 3) &&
      imuWrite(0x19, 9) && imuWrite(0x1B, 0) && imuWrite(0x1C, 0);
    if (imuReady) break;
  }
  imuPending = false;
  Serial.printf("IMU ready=%d address=0x%02x\n", imuReady, imuAddress);
}
void pollImu(uint32_t now) {
  if (!imuReady) {
    if (uint32_t(now-imuLastRetryMs) >= 2000) { imuLastRetryMs = now; initImu(); }
    return;
  }
  if (uint32_t(now-imuLastSampleMs) < 10) return;
  imuLastSampleMs = now;
  uint8_t raw[14];
  if (!imuRead(0x3B, raw, 14)) { imuReady = false; imuPending = false; return; }
  memcpy(imuPayload, &now, 4);
  for (int i = 0; i < 6; ++i) {
    int offset = i < 3 ? i * 2 : i * 2 + 2;
    int16_t value = int16_t((uint16_t(raw[offset]) << 8) | raw[offset+1]);
    float converted = i < 3 ? value * (9.80665f / 16384.0f) : value * (PI / (180.0f * 131.0f));
    memcpy(imuPayload+4+i*4, &converted, 4);
  }
  imuPending = true;  // Replace unsent samples rather than accumulate latency.
}

void sendStatus(uint32_t now) {
  static uint32_t lastMs = 0;
  if (uint32_t(now-lastMs) < 50) return;
  MotorState state;
  portENTER_CRITICAL(&telemetryMux); state = motorState; portEXIT_CRITICAL(&telemetryMux);
  uint8_t payload[53], offset = 0;
  auto append = [&](const void* value, uint8_t size) { memcpy(payload+offset, value, size); offset += size; };
  append(state.rpm, 8); append(state.targets, 8); append(state.encoders, 8);
  append(&batteryVoltage, 4); append(state.pwm, 4);
  uint8_t flags = 0;
  portENTER_CRITICAL(&commandMux);
  if (motion.valid) flags |= 1;
  if (motion.timed_out) flags |= 2;
  portEXIT_CRITICAL(&commandMux);
  if (batteryVoltage < 9.6f) flags |= 4;
  if (imuReady) flags |= 8;
  if (lidarPacketCount && uint32_t(now-lidarLastPacketMs) < 1000) flags |= 16;
  append(&flags, 1);
  uint32_t version = FIRMWARE_VERSION;
  append(&version, 4); append(&rxByteCount, 4); append(&rxValidCmdCount, 4);
  append(&rxBadChecksumCount, 4); append(&rxBadLenCount, 4);
  if (sendFrame(0x81, payload, offset)) lastMs = now;
}

void lidarFinishCurrentScan(uint32_t now) {
  // The on-board display remains useful even without a connected ROS host.
  if (scanAligned && !scanOverflow && writeCount >= 100) {
    frontDistance.update(scans[writeIndex], writeCount, now);
  }
  if (scanAligned && !scanOverflow && writeCount >= 100 && sendCount == 0) {
    sendIndex = writeIndex; writeIndex = 1-writeIndex;
    sendCount = writeCount; sendOffset = 0;
    sendScanId = nextScanId; sendStamp = writeStamp;
  } else if (scanAligned) { ++lidarDroppedScans; }
  ++nextScanId; writeCount = 0; writeStamp = now; scanAligned = true; scanOverflow = false;
}

void pollDisplay(uint32_t now) {
  if (!statusDisplay.ready()) return;
  if (statusDisplay.pending()) { statusDisplay.flushChunk(); return; }
  static uint32_t lastMs = 0;
  if (uint32_t(now-lastMs) < DISPLAY_UPDATE_MS) return;
  lastMs = now;
  MotorState state;
  portENTER_CRITICAL(&telemetryMux); state = motorState; portEXIT_CRITICAL(&telemetryMux);
  char line[32];
  statusDisplay.clear();
  statusDisplay.drawString(0, 0, "WP-SMART 20261115");
  if (!isfinite(batteryVoltage) || batteryVoltage <= 0) snprintf(line, sizeof(line), "BAT:--V");
  else snprintf(line, sizeof(line), "BAT:%.2fV", batteryVoltage);
  statusDisplay.drawString(0, 10, line);
  if (frontDistance.valid(now)) {
    snprintf(line, sizeof(line), "FRONT:%.2fm", frontDistance.range_mm*0.001f);
  } else snprintf(line, sizeof(line), "FRONT:--");
  statusDisplay.drawString(0, 20, line);
  snprintf(line, sizeof(line), "L ENC:%ld", long(state.encoders[0]));
  statusDisplay.drawString(0, 30, line);
  snprintf(line, sizeof(line), "R ENC:%ld", long(state.encoders[1]));
  statusDisplay.drawString(0, 40, line);
  bool lidarFresh = lidarPacketCount && uint32_t(now-lidarLastPacketMs) < 1000;
  snprintf(line, sizeof(line), "IMU:%s    LIDAR:%s", imuReady ? "OK" : "--", lidarFresh ? "OK" : "--");
  statusDisplay.drawString(0, 50, line);
  statusDisplay.present();
}
void lidarAppendScanPoint(float angle, uint16_t distance) {
  if (!scanAligned) return;
  if (writeCount >= MAX_SCAN_POINTS) { scanOverflow = true; return; }
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  uint16_t cdeg = uint16_t(angle*100+0.5f) % 36000;
  scans[writeIndex][writeCount++] = {cdeg, distance};
}
void sendScanFragment() {
  if (!sendCount) return;
  uint8_t count = min(uint16_t(12), uint16_t(sendCount-sendOffset));
  uint8_t payload[60];
  memcpy(payload, &sendStamp, 4); memcpy(payload+4, &sendScanId, 2);
  memcpy(payload+6, &sendCount, 2); memcpy(payload+8, &sendOffset, 2);
  payload[10] = count; payload[11] = sendOffset+count == sendCount ? 1 : 0;
  memcpy(payload+12, scans[sendIndex]+sendOffset, count*4);
  if (sendFrame(0x83, payload, 12+4*count)) {
    sendOffset += count;
    if (sendOffset == sendCount) sendCount = 0;
  }
}
void pcnt_encoder_init() {
  // ── 配置编码器引脚的内部上拉电阻 (极度重要，防止引脚悬空拾取电机 PWM 噪声) ──
  pinMode(ENC1_A_PIN, INPUT_PULLUP);
  pinMode(ENC1_B_PIN, INPUT_PULLUP);
  pinMode(ENC2_A_PIN, INPUT_PULLUP);
  pinMode(ENC2_B_PIN, INPUT_PULLUP);

  // ── 编码器 1 (Unit 0) ──
  pcnt_config_t pcnt_config_1a = {
      .pulse_gpio_num = ENC1_A_PIN,
      .ctrl_gpio_num = ENC1_B_PIN,
      .lctrl_mode = PCNT_MODE_REVERSE,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_DEC,
      .neg_mode = PCNT_COUNT_INC,
      .counter_h_lim = 30000,
      .counter_l_lim = -30000,
      .unit = PCNT_UNIT_0,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_1a);

  pcnt_config_t pcnt_config_1b = {
      .pulse_gpio_num = ENC1_B_PIN,
      .ctrl_gpio_num = ENC1_A_PIN,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_REVERSE,
      .pos_mode = PCNT_COUNT_DEC,
      .neg_mode = PCNT_COUNT_INC,
      .counter_h_lim = 30000,
      .counter_l_lim = -30000,
      .unit = PCNT_UNIT_0,
      .channel = PCNT_CHANNEL_1,
  };
  pcnt_unit_config(&pcnt_config_1b);

  // 设置硬件滤波器 (100 个 APB 周期, 滤除 1.25us 以下的高频毛刺，避免过度滤波吃掉有效脉冲)
  pcnt_set_filter_value(PCNT_UNIT_0, 100);
  pcnt_filter_enable(PCNT_UNIT_0);

  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  // ── 编码器 2 (Unit 1) ──
  pcnt_config_t pcnt_config_2a = {
      .pulse_gpio_num = ENC2_A_PIN,
      .ctrl_gpio_num = ENC2_B_PIN,
      .lctrl_mode = PCNT_MODE_REVERSE,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_DEC,
      .neg_mode = PCNT_COUNT_INC,
      .counter_h_lim = 30000,
      .counter_l_lim = -30000,
      .unit = PCNT_UNIT_1,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_2a);

  pcnt_config_t pcnt_config_2b = {
      .pulse_gpio_num = ENC2_B_PIN,
      .ctrl_gpio_num = ENC2_A_PIN,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_REVERSE,
      .pos_mode = PCNT_COUNT_DEC,
      .neg_mode = PCNT_COUNT_INC,
      .counter_h_lim = 30000,
      .counter_l_lim = -30000,
      .unit = PCNT_UNIT_1,
      .channel = PCNT_CHANNEL_1,
  };
  pcnt_unit_config(&pcnt_config_2b);

  pcnt_set_filter_value(PCNT_UNIT_1, 100);
  pcnt_filter_enable(PCNT_UNIT_1);

  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);
  pcnt_counter_resume(PCNT_UNIT_1);
}
void motor_init() {
  ledcSetup(LEDC_M1_IN1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcSetup(LEDC_M1_IN2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcSetup(LEDC_M2_IN1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcSetup(LEDC_M2_IN2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);

  ledcAttachPin(MOTOR1_IN1, LEDC_M1_IN1);
  ledcAttachPin(MOTOR1_IN2, LEDC_M1_IN2);
  ledcAttachPin(MOTOR2_IN1, LEDC_M2_IN1);
  ledcAttachPin(MOTOR2_IN2, LEDC_M2_IN2);

  ledcWrite(LEDC_M1_IN1, 0);
  ledcWrite(LEDC_M1_IN2, 0);
  ledcWrite(LEDC_M2_IN1, 0);
  ledcWrite(LEDC_M2_IN2, 0);
}
uint16_t lidarChecksum(uint8_t ct, uint8_t lsn, uint16_t fsa, uint16_t lsa, const uint16_t* samples) {
  uint16_t checksum = 0x55AA;
  checksum ^= (uint16_t)ct | ((uint16_t)lsn << 8);
  checksum ^= fsa;
  checksum ^= lsa;
  for (uint8_t i = 0; i < lsn; i++) {
    checksum ^= samples[i];
  }
  return checksum;
}
float lidarRawAngleDeg(uint16_t rawAngle) {
  return (float)(rawAngle >> 1) / 64.0f;
}
float lidarCorrectAngleDeg(float baseAngleDeg, uint16_t distanceMm) {
  if (distanceMm == 0) {
    return baseAngleDeg;
  }

  float correction = atan(21.8f * (155.3f - (float)distanceMm) / (155.3f * (float)distanceMm));
  float angle = baseAngleDeg + correction * 180.0f / PI;
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}
void handleLidarPacket(uint8_t ct, uint8_t lsn, uint16_t fsa, uint16_t lsa,
                       uint16_t cs, const uint16_t* samples) {
  if (lidarChecksum(ct, lsn, fsa, lsa, samples) != cs || !(fsa & 1) || !(lsa & 1)) {
    ++lidarBadChecksumCount; return;
  }
  float start = lidarRawAngleDeg(fsa), end = lidarRawAngleDeg(lsa);
  if (start >= 360 || end >= 360) { ++lidarBadChecksumCount; return; }
  uint32_t now = millis();
  if (lidarPacketCount && uint32_t(now-lidarLastPacketMs) > 1000) {
    scanAligned = false; writeCount = 0;
  }
  if (ct & 1) lidarFinishCurrentScan(now);
  float difference = end-start;
  if (difference < 0) difference += 360;
  for (uint8_t i = 0; i < lsn; ++i) {
    uint8_t flag = samples[i] & 3;
    uint16_t distance = samples[i] >> 2;
    if (!distance || flag >= 2) continue;
    float base = start + (lsn > 1 ? difference*i/(lsn-1) : 0);
    lidarAppendScanPoint(lidarCorrectAngleDeg(base, distance), distance);
  }
  ++lidarPacketCount; lidarLastPacketMs = now;
}
void pollLidarSerial() {
  enum LidarParseState {
    LIDAR_WAIT_HEAD_1,
    LIDAR_WAIT_HEAD_2,
    LIDAR_READ_CT,
    LIDAR_READ_LSN,
    LIDAR_READ_FSA_L,
    LIDAR_READ_FSA_H,
    LIDAR_READ_LSA_L,
    LIDAR_READ_LSA_H,
    LIDAR_READ_CS_L,
    LIDAR_READ_CS_H,
    LIDAR_READ_SAMPLE_L,
    LIDAR_READ_SAMPLE_H
  };

  static LidarParseState state = LIDAR_WAIT_HEAD_1;
  static uint8_t ct = 0;
  static uint8_t lsn = 0;
  static uint8_t sampleIndex = 0;
  static uint8_t sampleLow = 0;
  static uint16_t fsa = 0;
  static uint16_t lsa = 0;
  static uint16_t cs = 0;
  static uint16_t samples[LIDAR_MAX_SAMPLES];

  uint16_t bytesHandled = 0;
  while (LidarSerial.available() > 0 && bytesHandled < LIDAR_POLL_BYTES) {
    uint8_t b = (uint8_t)LidarSerial.read();
    bytesHandled++;

    switch (state) {
      case LIDAR_WAIT_HEAD_1:
        state = (b == 0xAA) ? LIDAR_WAIT_HEAD_2 : LIDAR_WAIT_HEAD_1;
        break;
      case LIDAR_WAIT_HEAD_2:
        if (b == 0x55) {
          state = LIDAR_READ_CT;
        } else {
          state = (b == 0xAA) ? LIDAR_WAIT_HEAD_2 : LIDAR_WAIT_HEAD_1;
        }
        break;
      case LIDAR_READ_CT:
        ct = b;
        state = LIDAR_READ_LSN;
        break;
      case LIDAR_READ_LSN:
        lsn = b;
        sampleIndex = 0;
        if (lsn > 0 && lsn <= LIDAR_MAX_SAMPLES) {
          state = LIDAR_READ_FSA_L;
        } else {
          lidarBadChecksumCount++;
          state = LIDAR_WAIT_HEAD_1;
        }
        break;
      case LIDAR_READ_FSA_L:
        fsa = b;
        state = LIDAR_READ_FSA_H;
        break;
      case LIDAR_READ_FSA_H:
        fsa |= (uint16_t)b << 8;
        state = LIDAR_READ_LSA_L;
        break;
      case LIDAR_READ_LSA_L:
        lsa = b;
        state = LIDAR_READ_LSA_H;
        break;
      case LIDAR_READ_LSA_H:
        lsa |= (uint16_t)b << 8;
        state = LIDAR_READ_CS_L;
        break;
      case LIDAR_READ_CS_L:
        cs = b;
        state = LIDAR_READ_CS_H;
        break;
      case LIDAR_READ_CS_H:
        cs |= (uint16_t)b << 8;
        state = LIDAR_READ_SAMPLE_L;
        break;
      case LIDAR_READ_SAMPLE_L:
        sampleLow = b;
        state = LIDAR_READ_SAMPLE_H;
        break;
      case LIDAR_READ_SAMPLE_H:
        samples[sampleIndex++] = (uint16_t)sampleLow | ((uint16_t)b << 8);
        if (sampleIndex >= lsn) {
          handleLidarPacket(ct, lsn, fsa, lsa, cs, samples);
          state = LIDAR_WAIT_HEAD_1;
        } else {
          state = LIDAR_READ_SAMPLE_L;
        }
        break;
    }
  }
}

void setup() {
  // Coast before peripherals initialize; PID applies braking at zero speed.
  for (int pin : {11, 12, 13, 14, 21}) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  Serial.begin(115200);
  HostSerial.setRxBufferSize(2048);
  HostSerial.begin(460800, SERIAL_8N1, 18, 17);
  LidarSerial.setRxBufferSize(4096);
  LidarSerial.begin(115200, SERIAL_8N1, 47, 48);
  pcnt_encoder_init(); motor_init();
  pidTicker.attach_ms(10, pid_update);
  Wire.begin(9, 10); Wire.setTimeOut(10); Wire.setClock(100000);
  initImu();
  analogReadResolution(12); analogSetPinAttenuation(1, ADC_11db);
  batteryVoltage = analogReadMilliVolts(1)*0.001f*5.1f*3.3f/2.5f;
  Serial.printf("OLED ready=%d\n", statusDisplay.begin());
  ledcSetup(4, 20000, 8); ledcAttachPin(21, 4); ledcWrite(4, 110);
  Serial.printf("WP serial firmware %lu, UART1=460800, ratio=64:1\n", FIRMWARE_VERSION);
}
void loop() {
  uint32_t now = millis();
  pollHost(); pollLidarSerial(); pollImu(now);
  sendStatus(millis());  // pollLidarSerial() may have advanced the packet timestamp.
  if (imuPending && sendFrame(0x82, imuPayload, sizeof(imuPayload))) imuPending = false;
  // Limited work per iteration. availableForWrite prevents blocking the receive path.
  for (int i = 0; i < 4; ++i) sendScanFragment();
  static uint32_t lastBattery = 0, lastDebug = 0;
  if (uint32_t(now-lastBattery) >= 200) {
    lastBattery = now;
    float voltage = analogReadMilliVolts(1)*0.001f*5.1f*3.3f/2.5f;
    batteryVoltage = batteryVoltage*0.9f+voltage*0.1f;
  }
  if (uint32_t(now-lastDebug) >= 1000) {
    lastDebug = now;
    Serial.printf("cmd=%lu imu=%d lidar=%lu bad=%lu dropped=%lu V=%.2f\n",
      rxValidCmdCount, imuReady, lidarPacketCount, lidarBadChecksumCount, lidarDroppedScans, batteryVoltage);
  }
  pollDisplay(millis());
  delay(1);
}
