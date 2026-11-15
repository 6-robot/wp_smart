/*
 * SSD1306 OLED + 双路电机编码器 + WS2812B + 电池电压监测 + LED指示灯 + 双路电机驱动
 * 
 * 开发板: ESP32-S3-WROOM-1 开发套件
 * OLED:   SSD1306 128x64 I2C (SDA=GPIO9, SCL=GPIO10, 地址=0x3C)
 * 编码器1: MT6701CT-ACD  A相=GPIO4, B相=GPIO5
 * 编码器2: MT6701CT-ACD  A相=GPIO6, B相=GPIO7
 * WS2812B: GPIO45, 6颗灯珠, RMT驱动
 * 电池ADC: GPIO1, 3S 18650 (9~12.6V), 分压电阻 82K/20K
 * LED指示灯: GPIO40, GPIO41, GPIO42
 * 电机1:  DRV8870 IN1=GPIO11, IN2=GPIO12
 * 电机2:  DRV8870 IN1=GPIO13, IN2=GPIO14
 * 按键:   GPIO8 (轻触按键, 低电平有效)
 * 参数:   1024 PPR, 4倍频解码 = 4096 计数/转
 * 减速比: 64:1 (电机轴:输出轴)
 *
 * 速度单位统一为减速机输出轴 RPM（转/分钟），方便直接观察车轮转速。
 */

#include <Arduino.h>
#include <driver/rmt.h>
#include <Ticker.h>
#include "driver/pcnt.h"
#include "ssd1306.h"


// ══════════════════════════════════════════════════════════════
//  硬件引脚定义
// ══════════════════════════════════════════════════════════════
// 编码器引脚
#define ENC1_A_PIN      4
#define ENC1_B_PIN      5
#define ENC2_A_PIN      6
#define ENC2_B_PIN      7

// 编码器参数
#define ENCODER_PPR     1024
#define COUNTS_PER_REV  (ENCODER_PPR * 4)  // 4096

// 减速比
#define REDUCTION_RATIO 64

// WS2812B 引脚及参数
#define WS2812B_PIN     45       // 数据引脚 GPIO45
#define NUM_LEDS        6        // 灯珠数量
#define RMT_CHANNEL     RMT_CHANNEL_0
#define RMT_CLK_DIV     2        // 80MHz / 2 = 40MHz -> 每tick 25ns

// WS2812B 时序 (单位: RMT tick, 1 tick = 25ns)
#define T0H_TICKS       16       // 400ns  = 16 * 25ns
#define T0L_TICKS       34       // 850ns  = 34 * 25ns
#define T1H_TICKS       32       // 800ns  = 32 * 25ns
#define T1L_TICKS       18       // 450ns  = 18 * 25ns

// 电池 ADC 引脚及参数
#define BATTERY_ADC_PIN   1         // GPIO1 (ADC1_CH0)
#define VOLTAGE_DIV_R1    82000.0f  // 上分压电阻 82K (连接电池正极)
#define VOLTAGE_DIV_R2    20000.0f  // 下分压电阻 20K (连接GND)
// 分压公式: Vadc = Vbat * R2 / (R1 + R2)
//           Vbat = Vadc * (R1 + R2) / R2 = Vadc * 5.1
#define VOLTAGE_DIV_RATIO ((VOLTAGE_DIV_R1 + VOLTAGE_DIV_R2) / VOLTAGE_DIV_R2)  // 5.1
#define BATTERY_FILTER_ALPHA 0.1f  // 一阶低通滤波系数 (越小越平滑)
#define LOW_BATTERY_THRESHOLD 9.6f // 低电压报警阈值 (V)

// LED 指示灯引脚
#define LED_PIN_1         40       // GPIO40 LED指示灯
#define LED_PIN_2         41       // GPIO41 LED指示灯
#define LED_PIN_3         42       // GPIO42 LED指示灯

// 电机驱动引脚 (DRV8870)
#define MOTOR1_IN1        11       // GPIO11 -> DRV8870 #1 IN1
#define MOTOR1_IN2        12       // GPIO12 -> DRV8870 #1 IN2
#define MOTOR2_IN1        13       // GPIO13 -> DRV8870 #2 IN1
#define MOTOR2_IN2        14       // GPIO14 -> DRV8870 #2 IN2

// 按键引脚
#define BUTTON_PIN        8        // GPIO8 轻触按键 (低电平有效)
#define BUTTON_DEBOUNCE_MS 300     // 按键去抖时间 (ms)

// 电机 PWM 参数
#define MOTOR_PWM_FREQ    20000    // 20kHz (DRV8870 推荐)
#define MOTOR_PWM_RES     8        // 8-bit 分辨率 (0~255)

// Raspberry Pi UART and differential drive parameters
#define PI_UART_BAUD      115200
#define PI_UART_TX_PIN    17       // ESP32 TX -> Raspberry Pi RX
#define PI_UART_RX_PIN    18       // ESP32 RX <- Raspberry Pi TX
#define WHEEL_DIAMETER_M  0.0566f
#define WHEEL_BASE_M      0.195f
#define MAX_LINEAR_MPS    0.50f
#define MAX_ANGULAR_RADPS 2.50f
#define CMD_TIMEOUT_MS    2000
#define STATUS_PERIOD_MS  50
#define FIRMWARE_VERSION  2026061505UL
#define MOTOR1_SIGN       1
#define MOTOR2_SIGN       -1

// Binary protocol: AA 55 type len payload checksum(sum type..payload low 8 bits)
#define FRAME_HEAD_1      0xAA
#define FRAME_HEAD_2      0x55
#define MSG_CMD_VEL       0x01
#define MSG_STATUS        0x81
#define PROTO_MAX_PAYLOAD 64

// ══════════════════════════════════════════════════════════════
//  PID 速度闭环控制 (使用 ESP32 硬件定时器, 10ms 伺服周期)
// ══════════════════════════════════════════════════════════════
// 架构说明:
//   PID 运行在 esp_timer 高优先级任务中 (通过 Ticker 封装),
//   以精确 10ms 周期调度, 不受主循环中 I2C/RMT/ADC 等慢速操作影响。
//   主循环 loop() 仅负责显示、LED、按键等非实时任务。
//
struct PIDController {
  float kp;
  float ki;
  float target;      // 目标速度 (单位: 脉冲/秒)
  float integral;    // 积分累加值 (单位: 脉冲)
  float prev_error;  // 上一次误差
  float lastOutput;  // 上一次 PWM 输出 (用于自动极性检测)
};

// 编码器方向 (变量, 自动极性检测会修改)
volatile int motor1EncDir = 1;
volatile int motor2EncDir = 1;
volatile bool motor1DirCalibrated = false;
volatile bool motor2DirCalibrated = false;
unsigned long calibStartTimeMs = 0;

// ── 输出轴目标转速 (单位：RPM，转/分钟) ──
#define DEMO_OUTPUT_RPM  5.0f     // 演示模式默认转速，可在此修改

#define SPEED_FILTER_ALPHA   0.6f       // EMA 滤波系数 (增大以减少相位延迟)
//#define KICK_DURATION_MS     250         // 启动冲击时长
//#define KICK_PWM             220         // 启动冲击占空比
#define PID_MAX_I_OUT        80.0f     // 积分项最大输出限制 (±100 PWM)

// PI 控制器 —— 统一使用输出轴 RPM 作为计算单位，彻底解决数值缩放问题
// kp=1.5: 10 RPM 误差 -> 15 PWM 输出 (温和响应)
// ki=10.0: 1 RPM 误差 -> 1秒后产生 10 PWM 修正
PIDController pidM1 = {1.0f, 12.0f, 0.0f, 0.0f, 0.0f, 0.0f};
PIDController pidM2 = {1.0f, 12.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 工具函数：直接返回 RPM
inline float outputRPM_to_ticks(float rpm) 
{
    return rpm;
}

// 硬件定时器 (Ticker 封装 esp_timer, 高优先级独立于 loop)
Ticker pidTicker;

// PID 输出变量 (volatile: 定时器回调写, 主循环读)
volatile float speedM1 = 0.0f;   // 滤波后速度 (脉冲/秒)
volatile float speedM2 = 0.0f;
volatile int16_t pwmOutM1 = 0;   // 当前 PWM 输出
volatile int16_t pwmOutM2 = 0;

// Open-loop mode is reserved for internal testing; ROS2 UART uses closed-loop RPM targets.
volatile bool openLoopMode = false;
volatile int16_t openLoopPWM1 = 0;
volatile int16_t openLoopPWM2 = 0;

volatile bool commTimedOut = false;
volatile bool hostConnected = false;
unsigned long lastCmdVelMs = 0;
float lastLinearCmdMps = 0.0f;
float lastAngularCmdRadps = 0.0f;
uint32_t lastRxValidCmdForTimeout = 0;
unsigned long lastCmdActivityMs = 0;
volatile uint32_t rxByteCount = 0;
volatile uint32_t rxValidCmdCount = 0;
volatile uint32_t rxBadChecksumCount = 0;
volatile uint32_t rxBadLenCount = 0;

// 自动校准消息标志 (定时器回调设置, 主循环打印)
volatile uint8_t calibMsgFlag = 0;  // 0=无, 1=M1校正, 2=M2校正, 3=M1正常, 4=M2正常

// LEDC 通道分配
#define LEDC_M1_IN1       0
#define LEDC_M1_IN2       1
#define LEDC_M2_IN1       2
#define LEDC_M2_IN2       3

// ══════════════════════════════════════════════════════════════
//  WS2812B 颜色结构体与缓冲区
// ══════════════════════════════════════════════════════════════
typedef struct {
  uint8_t r, g, b;
} RGB;

RGB leds[NUM_LEDS];

// ══════════════════════════════════════════════════════════════
//  编码器计数变量
// ══════════════════════════════════════════════════════════════
// 编码器 1
volatile int32_t enc1Count = 0;

// 编码器 2
volatile int32_t enc2Count = 0;

// ══════════════════════════════════════════════════════════════
//  电池电压变量
// ══════════════════════════════════════════════════════════════
float batteryVoltage = 0.0f;        // 滤波后电池电压
float lastDisplayedVoltage = -1.0f; // 上次显示的电压值

// ══════════════════════════════════════════════════════════════
//  电机演示状态机
// ══════════════════════════════════════════════════════════════
enum MotorDemoState {
  DEMO_IDLE,       // 空闲, 等待按键
  DEMO_FORWARD,    // 直行 2s
  DEMO_LEFT,       // 左转 2s
  DEMO_RIGHT,      // 右转 2s
  DEMO_REVERSE     // 后退 2s
};

MotorDemoState demoState = DEMO_IDLE;
unsigned long  demoStepStartMs = 0;  // 当前步骤开始时间
#define DEMO_STEP_DURATION 5000      // 每步持续时间 (ms)

// ══════════════════════════════════════════════════════════════
//  PCNT 硬件编码器初始化 (带毛刺滤波)
// ══════════════════════════════════════════════════════════════
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

// ══════════════════════════════════════════════════════════════
//  WS2812B RMT 驱动函数
// ══════════════════════════════════════════════════════════════

void ws2812_init() {
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)WS2812B_PIN, RMT_CHANNEL);
  config.clk_div = RMT_CLK_DIV;
  config.mem_block_num = 3;
  rmt_config(&config);
  rmt_driver_install(RMT_CHANNEL, 0, 0);
  Serial.println("[WS2812B] RMT driver installed on GPIO45");
}

// 设置单个 bit 的 RMT item
static void ws2812_set_bit(rmt_item32_t* item, uint8_t bit_val) {
  if (bit_val) {
    item->duration0 = T1H_TICKS;
    item->level0    = 1;
    item->duration1 = T1L_TICKS;
    item->level1    = 0;
  } else {
    item->duration0 = T0H_TICKS;
    item->level0    = 1;
    item->duration1 = T0L_TICKS;
    item->level1    = 0;
  }
}

// 将一个字节写入 rmt_items (8 个 item)
static void ws2812_set_byte(rmt_item32_t* items, uint8_t byte_val) {
  for (int i = 7; i >= 0; i--) {
    ws2812_set_bit(items++, (byte_val >> i) & 1);
  }
}

// 刷新所有 LED (发送数据)
void ws2812_show() {
  const int totalItems = NUM_LEDS * 24;
  rmt_item32_t items[totalItems];

  for (int led = 0; led < NUM_LEDS; led++) {
    int offset = led * 24;
    // WS2812B 顺序: GRB
    ws2812_set_byte(&items[offset +  0], leds[led].g);
    ws2812_set_byte(&items[offset +  8], leds[led].r);
    ws2812_set_byte(&items[offset + 16], leds[led].b);
  }

  rmt_write_items(RMT_CHANNEL, items, totalItems, true);
}

// 设置单颗灯珠颜色
void setLED(int index, uint8_t r, uint8_t g, uint8_t b) {
  if (index >= 0 && index < NUM_LEDS) {
    leds[index].r = r;
    leds[index].g = g;
    leds[index].b = b;
  }
}

// 设置所有灯珠为同一颜色
void setAllLEDs(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].r = r;
    leds[i].g = g;
    leds[i].b = b;
  }
}

// 关闭所有灯珠
void clearLEDs() {
  setAllLEDs(0, 0, 0);
  ws2812_show();
}

// 设置单颗灯珠的 HSV 颜色 (h: 0~360, s: 0~255, v: 0~255)
void setHSV(int index, uint16_t h, uint8_t s, uint8_t v) {
  uint8_t r, g, b;
  uint8_t region = h / 60;
  uint8_t remainder = (h % 60) * 255 / 60;
  uint8_t p = (v * (255 - s)) >> 8;
  uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (region) {
    case 0:  r = v; g = t; b = p; break;
    case 1:  r = q; g = v; b = p; break;
    case 2:  r = p; g = v; b = t; break;
    case 3:  r = p; g = q; b = v; break;
    case 4:  r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
  }
  setLED(index, r, g, b);
}

// 彩虹渐变效果 (非阻塞版, 在 loop 中调用)
void ws2812_rainbow_update() {
  static uint16_t hue_offset = 0;
  for (int i = 0; i < NUM_LEDS; i++) {
    uint16_t hue = (hue_offset + i * 60) % 360;
    setHSV(i, hue, 255, 50); // 亮度限制在50，节能护眼
  }
  ws2812_show();
  hue_offset = (hue_offset + 2) % 360;
}


// 低电压红灯慢闪效果 (非阻塞, 使用 sin 呼吸式渐变)
void ws2812_low_battery_flash() {
  float phase = (float)(millis() % 2000) / 2000.0f;
  float brightness = (sin(phase * 2.0f * PI - PI / 2.0f) + 1.0f) / 2.0f;
  uint8_t r = (uint8_t)(brightness * 60.0f);
  setAllLEDs(r, 0, 0);
  ws2812_show();
}

// ══════════════════════════════════════════════════════════════
//  电机驱动函数 (DRV8870)
// ══════════════════════════════════════════════════════════════

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

void setMotor1(int16_t speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    ledcWrite(LEDC_M1_IN1, speed);
    ledcWrite(LEDC_M1_IN2, 0);
  } else if (speed < 0) {
    ledcWrite(LEDC_M1_IN1, 0);
    ledcWrite(LEDC_M1_IN2, -speed);
  } else {
    ledcWrite(LEDC_M1_IN1, 0);
    ledcWrite(LEDC_M1_IN2, 0);
  }
}

void setMotor2(int16_t speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    ledcWrite(LEDC_M2_IN1, speed);
    ledcWrite(LEDC_M2_IN2, 0);
  } else if (speed < 0) {
    ledcWrite(LEDC_M2_IN1, 0);
    ledcWrite(LEDC_M2_IN2, -speed);
  } else {
    ledcWrite(LEDC_M2_IN1, 0);
    ledcWrite(LEDC_M2_IN2, 0);
  }
}

// ── PID 闭环更新 (Ticker 回调, 每 10ms 自动执行) ──
void pid_update() {

  const float dt = 0.01f;

  int16_t pcnt1 = 0, pcnt2 = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &pcnt1);
  pcnt_get_counter_value(PCNT_UNIT_1, &pcnt2);

  static int16_t last_pcnt1 = 0, last_pcnt2 = 0;
  int32_t delta1 = (int32_t)pcnt1 - last_pcnt1;
  int32_t delta2 = (int32_t)pcnt2 - last_pcnt2;

  // PCNT h_lim=30000, l_lim=-30000, 到限清零, 修正值 = 限值本身
  if (delta1 < -15000) delta1 += 30000;
  else if (delta1 > 15000) delta1 -= 30000;
  if (delta2 < -15000) delta2 += 30000;
  else if (delta2 > 15000) delta2 -= 30000;

  last_pcnt1 = pcnt1;
  last_pcnt2 = pcnt2;

  enc1Count += delta1;
  enc2Count += delta2;

  float rawSpeed1 = (float)delta1 * 100.0f;
  float rawSpeed2 = (float)delta2 * 100.0f;

  float dirSpeed1 = rawSpeed1 * motor1EncDir;
  float dirSpeed2 = rawSpeed2 * motor2EncDir;

  // ══ 自适应速度滤波 ══
  // 将脉冲/秒转换为输出轴 RPM
  float rpm1 = dirSpeed1 * 60.0f / (REDUCTION_RATIO * COUNTS_PER_REV);
  float rpm2 = dirSpeed2 * 60.0f / (REDUCTION_RATIO * COUNTS_PER_REV);
  
  float tgtRPM1 = fabs(pidM1.target);
  float tgtRPM2 = fabs(pidM2.target);
  
  // 移除迟缓的滑动平均，仅使用轻微的 EMA 滤波，大幅降低闭环相位延迟
  speedM1 = speedM1 * (1.0f - SPEED_FILTER_ALPHA) + rpm1 * SPEED_FILTER_ALPHA;
  speedM2 = speedM2 * (1.0f - SPEED_FILTER_ALPHA) + rpm2 * SPEED_FILTER_ALPHA;

  // ══ 电机控制 ══
  int16_t pwm1 = 0, pwm2 = 0;

  if (openLoopMode) {
    pwm1 = openLoopPWM1;
    pwm2 = openLoopPWM2;
    pidM1.integral = 0;
    pidM2.integral = 0;
  } else {
    // ── 电机 1：前馈 + PI 闭环 + 启动脉冲 ──
    static bool startup1 = false;
    static unsigned long startTime1 = 0;
    static float lastTgt1 = 0;  // 移至此处, 确保 target=0 时能重置
    
    if (pidM1.target == 0.0f) {
      pwm1 = 0;
      pidM1.integral = 0;
      startup1 = false;
      lastTgt1 = 0;
    } else {
      if (lastTgt1 == 0 && pidM1.target != 0) {
        startup1 = true;
        startTime1 = millis();
        pidM1.integral = 0;
      }
      lastTgt1 = pidM1.target;
      
      // 前馈模型修正: 基于真实测量的拟合 (死区 107, 斜率 0.5 PWM/RPM)
      float ff1 = 0;
      if (tgtRPM1 > 0) {
        ff1 = 107.0f + tgtRPM1 * 0.5f;
        // 动态静摩擦补偿：速度极低时额外给点推力，彻底解决低速卡顿 (Stick-Slip)
        if (fabs(speedM1) < 2.0f) {
           ff1 += 20.0f;
        }
        ff1 = constrain(ff1, 107.0f, 255.0f);
      }
      // 如果目标为负，ff 取反（前馈也反转）
      if (pidM1.target < 0) ff1 = -ff1;
      
      // 移除强制启动脉冲延时，完全交给精准前馈和 PI 控制，防止高转速超调
      if (startup1) {
        startup1 = false;
        // 预置积分，使切换到 PI 时输出平滑衔接
        pidM1.integral = 0;  // 前馈已非常精确，积分从 0 开始微调即可
      }
      
      // PI 控制（kp 即时纠偏 + ki 消除静差）
      float error1 = pidM1.target - speedM1;
      
      // Anti-windup：用上一次实际输出值判断饱和
      float prevOut1 = pidM1.lastOutput;
      if ((prevOut1 > -255 && prevOut1 < 255) || (prevOut1 >= 255 && error1 < 0) || (prevOut1 <= -255 && error1 > 0)) {
        pidM1.integral += error1 * dt;
        pidM1.integral = constrain(pidM1.integral, -PID_MAX_I_OUT / pidM1.ki, PID_MAX_I_OUT / pidM1.ki);
      }
      
      float output1 = ff1 + pidM1.kp * error1 + pidM1.ki * pidM1.integral;
      pwm1 = (int16_t)constrain(output1, -255.0f, 255.0f);
    }

    // ── 电机 2：同上 ──
    static bool startup2 = false;
    static unsigned long startTime2 = 0;
    static float lastTgt2 = 0;
    
    if (pidM2.target == 0.0f) {
      pwm2 = 0;
      pidM2.integral = 0;
      startup2 = false;
      lastTgt2 = 0;
    } else {
      if (lastTgt2 == 0 && pidM2.target != 0) {
        startup2 = true;
        startTime2 = millis();
        pidM2.integral = 0;
      }
      lastTgt2 = pidM2.target;
      
      float ff2 = 0;
      if (tgtRPM2 > 0) {
        ff2 = 107.0f + tgtRPM2 * 0.5f;
        if (fabs(speedM2) < 2.0f) {
           ff2 += 20.0f;
        }
        ff2 = constrain(ff2, 107.0f, 255.0f);
      }
      if (pidM2.target < 0) ff2 = -ff2;
      
      if (startup2) {
        startup2 = false;
        pidM2.integral = 0;
      }
      
      float error2 = pidM2.target - speedM2;
      float prevOut2 = pidM2.lastOutput;
      if ((prevOut2 > -255 && prevOut2 < 255) || (prevOut2 >= 255 && error2 < 0) || (prevOut2 <= -255 && error2 > 0)) {
        pidM2.integral += error2 * dt;
        pidM2.integral = constrain(pidM2.integral, -PID_MAX_I_OUT / pidM2.ki, PID_MAX_I_OUT / pidM2.ki);
      }
      
      float output2 = ff2 + pidM2.kp * error2 + pidM2.ki * pidM2.integral;
      pwm2 = (int16_t)constrain(output2, -255.0f, 255.0f);
    }
  }

  pidM1.lastOutput = (float)pwm1;
  pidM2.lastOutput = (float)pwm2;
  pwmOutM1 = pwm1;
  pwmOutM2 = pwm2;

  // 写电机驱动: PWM=0 时触发 DRV8870 硬件刹车 (IN1=1, IN2=1) 实现完美自锁
  if (pwm1 > 0)      { ledcWrite(LEDC_M1_IN1, pwm1);  ledcWrite(LEDC_M1_IN2, 0); }
  else if (pwm1 < 0) { ledcWrite(LEDC_M1_IN1, 0);     ledcWrite(LEDC_M1_IN2, -pwm1); }
  else               { ledcWrite(LEDC_M1_IN1, 255);   ledcWrite(LEDC_M1_IN2, 255); }

  if (pwm2 > 0)      { ledcWrite(LEDC_M2_IN1, pwm2);  ledcWrite(LEDC_M2_IN2, 0); }
  else if (pwm2 < 0) { ledcWrite(LEDC_M2_IN1, 0);     ledcWrite(LEDC_M2_IN2, -pwm2); }
  else               { ledcWrite(LEDC_M2_IN1, 255);   ledcWrite(LEDC_M2_IN2, 255); }
}

// ── 便捷运动函数 (参数为输出轴 RPM) ──
void motorsForward(float rpm) {
  float t = outputRPM_to_ticks(rpm);
  pidM1.target = t * MOTOR1_SIGN;
  pidM2.target = t * MOTOR2_SIGN;
}
void motorsReverse(float rpm) {
  float t = outputRPM_to_ticks(rpm);
  pidM1.target = -t * MOTOR1_SIGN;
  pidM2.target = -t * MOTOR2_SIGN;
}
void motorsLeft(float rpm) {
  float t = outputRPM_to_ticks(rpm);
  pidM1.target = -t * MOTOR1_SIGN;
  pidM2.target = t * MOTOR2_SIGN;
}
void motorsRight(float rpm) {
  float t = outputRPM_to_ticks(rpm);
  pidM1.target = t * MOTOR1_SIGN;
  pidM2.target = -t * MOTOR2_SIGN;
}
void motorsStop() {
  pidM1.target = 0;
  pidM2.target = 0;
}

uint8_t protocolChecksum(uint8_t type, uint8_t len, const uint8_t* payload) {
  uint8_t sum = type + len;
  for (uint8_t i = 0; i < len; i++) {
    sum += payload[i];
  }
  return sum;
}

float readFloatLE(const uint8_t* data) {
  float value;
  memcpy(&value, data, sizeof(value));
  return value;
}

void appendPayloadBytes(uint8_t* payload, uint8_t& offset, const void* value, uint8_t size) {
  memcpy(payload + offset, value, size);
  offset += size;
}

void sendProtocolFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  Serial1.write((uint8_t)FRAME_HEAD_1);
  Serial1.write((uint8_t)FRAME_HEAD_2);
  Serial1.write(type);
  Serial1.write(len);
  if (len > 0) {
    Serial1.write(payload, len);
  }
  Serial1.write(protocolChecksum(type, len, payload));
}

void setDiffDriveCommand(float linearMps, float angularRadps) {
  linearMps = constrain(linearMps, -MAX_LINEAR_MPS, MAX_LINEAR_MPS);
  angularRadps = constrain(angularRadps, -MAX_ANGULAR_RADPS, MAX_ANGULAR_RADPS);

  const float leftMps = linearMps - angularRadps * WHEEL_BASE_M * 0.5f;
  const float rightMps = linearMps + angularRadps * WHEEL_BASE_M * 0.5f;
  const float wheelCircumference = PI * WHEEL_DIAMETER_M;
  const float leftRpm = leftMps * 60.0f / wheelCircumference;
  const float rightRpm = rightMps * 60.0f / wheelCircumference;

  pidM1.target = outputRPM_to_ticks(leftRpm * MOTOR1_SIGN);
  pidM2.target = outputRPM_to_ticks(rightRpm * MOTOR2_SIGN);
  openLoopMode = false;
  lastLinearCmdMps = linearMps;
  lastAngularCmdRadps = angularRadps;
}

void handleCommandFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  if (type != MSG_CMD_VEL) {
    return;
  }
  if (len != 8) {
    rxBadLenCount++;
    return;
  }

  float linearMps = readFloatLE(payload);
  float angularRadps = readFloatLE(payload + 4);
  if (!isfinite(linearMps) || !isfinite(angularRadps)) {
    return;
  }

  setDiffDriveCommand(linearMps, angularRadps);
  rxValidCmdCount++;
  lastCmdVelMs = millis();
  hostConnected = true;
  commTimedOut = false;
}

void pollPiSerial() {
  enum ParseState {
    WAIT_HEAD_1,
    WAIT_HEAD_2,
    READ_TYPE,
    READ_LEN,
    READ_PAYLOAD,
    READ_CHECKSUM
  };

  static ParseState state = WAIT_HEAD_1;
  static uint8_t type = 0;
  static uint8_t len = 0;
  static uint8_t pos = 0;
  static uint8_t payload[PROTO_MAX_PAYLOAD];

  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();
    rxByteCount++;
    switch (state) {
      case WAIT_HEAD_1:
        state = (b == FRAME_HEAD_1) ? WAIT_HEAD_2 : WAIT_HEAD_1;
        break;
      case WAIT_HEAD_2:
        if (b == FRAME_HEAD_2) {
          state = READ_TYPE;
        } else {
          state = (b == FRAME_HEAD_1) ? WAIT_HEAD_2 : WAIT_HEAD_1;
        }
        break;
      case READ_TYPE:
        type = b;
        state = READ_LEN;
        break;
      case READ_LEN:
        len = b;
        pos = 0;
        if (len <= PROTO_MAX_PAYLOAD) {
          state = (len == 0 ? READ_CHECKSUM : READ_PAYLOAD);
        } else {
          rxBadLenCount++;
          state = WAIT_HEAD_1;
        }
        break;
      case READ_PAYLOAD:
        payload[pos++] = b;
        if (pos >= len) {
          state = READ_CHECKSUM;
        }
        break;
      case READ_CHECKSUM:
        if (b == protocolChecksum(type, len, payload)) {
          handleCommandFrame(type, payload, len);
        } else {
          rxBadChecksumCount++;
        }
        state = WAIT_HEAD_1;
        break;
    }
  }
}

void checkCommandTimeout(unsigned long now) {
  noInterrupts();
  uint32_t validCmdCount = rxValidCmdCount;
  interrupts();

  if (validCmdCount != lastRxValidCmdForTimeout) {
    lastRxValidCmdForTimeout = validCmdCount;
    lastCmdActivityMs = now;
    commTimedOut = false;
    return;
  }

  if (lastCmdActivityMs == 0) {
    return;
  }

  if (pidM1.target == 0.0f && pidM2.target == 0.0f) {
    return;
  }
  if (now - lastCmdActivityMs > CMD_TIMEOUT_MS) {
    motorsStop();
    openLoopMode = false;
    openLoopPWM1 = 0;
    openLoopPWM2 = 0;
    commTimedOut = true;
  }
}

void sendRobotStatus() {
  uint8_t payload[64];
  uint8_t offset = 0;

  noInterrupts();
  int32_t c1 = enc1Count;
  int32_t c2 = enc2Count;
  float s1 = speedM1;
  float s2 = speedM2;
  int16_t p1 = pwmOutM1;
  int16_t p2 = pwmOutM2;
  uint32_t rxBytes = rxByteCount;
  uint32_t rxValid = rxValidCmdCount;
  uint32_t rxBadChecksum = rxBadChecksumCount;
  uint32_t rxBadLen = rxBadLenCount;
  interrupts();

  uint32_t firmwareVersion = FIRMWARE_VERSION;
  float t1 = pidM1.target;
  float t2 = pidM2.target;
  uint8_t status = 0;
  if (hostConnected) status |= 0x01;
  if (commTimedOut) status |= 0x02;
  if (batteryVoltage > 0.5f && batteryVoltage < LOW_BATTERY_THRESHOLD) status |= 0x04;

  appendPayloadBytes(payload, offset, &s1, sizeof(s1));
  appendPayloadBytes(payload, offset, &s2, sizeof(s2));
  appendPayloadBytes(payload, offset, &t1, sizeof(t1));
  appendPayloadBytes(payload, offset, &t2, sizeof(t2));
  appendPayloadBytes(payload, offset, &c1, sizeof(c1));
  appendPayloadBytes(payload, offset, &c2, sizeof(c2));
  appendPayloadBytes(payload, offset, &batteryVoltage, sizeof(batteryVoltage));
  appendPayloadBytes(payload, offset, &p1, sizeof(p1));
  appendPayloadBytes(payload, offset, &p2, sizeof(p2));
  appendPayloadBytes(payload, offset, &status, sizeof(status));
  appendPayloadBytes(payload, offset, &firmwareVersion, sizeof(firmwareVersion));
  appendPayloadBytes(payload, offset, &rxBytes, sizeof(rxBytes));
  appendPayloadBytes(payload, offset, &rxValid, sizeof(rxValid));
  appendPayloadBytes(payload, offset, &rxBadChecksum, sizeof(rxBadChecksum));
  appendPayloadBytes(payload, offset, &rxBadLen, sizeof(rxBadLen));

  sendProtocolFrame(MSG_STATUS, payload, offset);
}

// ══════════════════════════════════════════════════════════════
//  电池电压 ADC 读取
// ══════════════════════════════════════════════════════════════

void battery_adc_init() {
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
  Serial.println("[Battery] ADC initialized on GPIO1 (using factory calibration)");
  Serial.printf("[Battery] Divider ratio: %.2f  (R1=82K, R2=20K)\n", VOLTAGE_DIV_RATIO);
  Serial.printf("[Battery] Formula: Vbat = Vadc * %.1f\n", VOLTAGE_DIV_RATIO);
}

float readBatteryVoltage() {
  int32_t sum_mV = 0;
  const int samples = 32;
  for (int i = 0; i < samples; i++) {
    sum_mV += analogReadMilliVolts(BATTERY_ADC_PIN);
  }
  float adcVoltage_mV = (float)sum_mV / (float)samples;
  float adcVoltage = adcVoltage_mV / 1000.0f;
  float vBat = adcVoltage * VOLTAGE_DIV_RATIO * 3.3/2.5;
  return vBat;
}

// ══════════════════════════════════════════════════════════════
//  OLED
// ══════════════════════════════════════════════════════════════
SSD1306 oled(0x3C, 9, 10);

float calcAngle(int32_t count) {
  int32_t mod = count % COUNTS_PER_REV;
  if (mod < 0) mod += COUNTS_PER_REV;
  return (float)mod * 360.0f / (float)COUNTS_PER_REV;
}

int32_t lastC1 = -99999, lastC2 = -99999;
float   lastA1 = -1.0f,  lastA2 = -1.0f;

void drawStaticUI() {
  char fwBuf[22];
  oled.clear();
  snprintf(fwBuf, sizeof(fwBuf), "ROS2 FW %lu", FIRMWARE_VERSION);
  oled.drawString(8, 0, fwBuf);
  oled.drawLine(0, 9, 127, 9);
  oled.drawString(0, 12, "ENC1 P:");
  oled.drawString(0, 22, "     A:");
  oled.drawLine(0, 31, 127, 31);
  oled.drawString(0, 34, "ENC2 P:");
  oled.drawString(0, 44, "     A:");
  oled.drawLine(0, 53, 127, 53);
  oled.drawString(0, 56, "BAT: --.-V");
  oled.display();
}

void updateDisplay(int32_t c1, float a1, int32_t c2, float a2, float vBat) {
  char buf[22];
  bool needRefresh = false;

  if (c1 != lastC1) {
    oled.fillRect(48, 12, 80, 8, false);
    snprintf(buf, sizeof(buf), "%ld", (long)c1);
    oled.drawString(48, 12, buf);
    lastC1 = c1;
    needRefresh = true;
  }
  if (a1 != lastA1) {
    oled.fillRect(48, 22, 80, 8, false);
    int ai = (int)a1; int ad = abs((int)((a1 - ai) * 10));
    snprintf(buf, sizeof(buf), "%d.%d deg", ai, ad);
    oled.drawString(48, 22, buf);
    lastA1 = a1;
    needRefresh = true;
  }

  if (c2 != lastC2) {
    oled.fillRect(48, 34, 80, 8, false);
    snprintf(buf, sizeof(buf), "%ld", (long)c2);
    oled.drawString(48, 34, buf);
    lastC2 = c2;
    needRefresh = true;
  }
  if (a2 != lastA2) {
    oled.fillRect(48, 44, 80, 8, false);
    int ai = (int)a2; int ad = abs((int)((a2 - ai) * 10));
    snprintf(buf, sizeof(buf), "%d.%d deg", ai, ad);
    oled.drawString(48, 44, buf);
    lastA2 = a2;
    needRefresh = true;
  }

  if (lastDisplayedVoltage < 0 || fabs(vBat - lastDisplayedVoltage) >= 0.05f) {
    oled.fillRect(0, 56, 128, 8, false);
    int vInt = (int)vBat;
    int vDec = abs((int)((vBat - vInt) * 10));
    snprintf(buf, sizeof(buf), "BAT:%d.%dV", vInt, vDec);
    if (vBat >= 12.0f) {
      snprintf(buf, sizeof(buf), "BAT:%d.%dV [FULL]", vInt, vDec);
    } else if (vBat >= 10.5f) {
      snprintf(buf, sizeof(buf), "BAT:%d.%dV [OK]", vInt, vDec);
    } else if (vBat >= 9.6f) {
      snprintf(buf, sizeof(buf), "BAT:%d.%dV [LOW!]", vInt, vDec);
    } else {
      snprintf(buf, sizeof(buf), "BAT:%d.%dV [CRIT]", vInt, vDec);
    }
    oled.drawString(0, 56, buf);
    lastDisplayedVoltage = vBat;
    needRefresh = true;
  }
  if (needRefresh) oled.display();
}

// ══════════════════════════════════════════════════════════════
//  Arduino 主程序
// ══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial1.begin(PI_UART_BAUD, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);
  delay(200);
  Serial.println("================================");
  Serial.println(" Dual Encoder + OLED + WS2812B");
  Serial.println(" + Battery Monitor + LED Indicator");
  Serial.println(" ENC1: A=4, B=5");
  Serial.println(" ENC2: A=6, B=7");
  Serial.println(" WS2812B: GPIO45, 6 LEDs");
  Serial.println(" Battery ADC: GPIO1");
  Serial.println(" Divider: 82K / 20K (ratio=5.1)");
  Serial.println(" LED Indicators: GPIO40,41,42");
  Serial.printf(" Pi UART: TX=GPIO%d RX=GPIO%d Baud=%d\n", PI_UART_TX_PIN, PI_UART_RX_PIN, PI_UART_BAUD);
  Serial.printf(" Firmware: %lu\n", FIRMWARE_VERSION);
  Serial.println("================================");
  Serial.printf("[Speed Unit] All speeds in output shaft RPM (reduction %d:1)\n", REDUCTION_RATIO);

  pcnt_encoder_init();
  Serial.println("[OK] PCNT Hardware Encoders initialized (with 1023 APB clock glitch filter).");

  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);
  digitalWrite(LED_PIN_1, LOW);
  digitalWrite(LED_PIN_2, LOW);
  digitalWrite(LED_PIN_3, LOW);
  Serial.println("[OK] LED indicators (GPIO40,41,42) initialized.");

  motor_init();
  Serial.println("[OK] Motor driver initialized (M1: GPIO11/12, M2: GPIO13/14).");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("[OK] Button on GPIO8 initialized.");

  memset(leds, 0, sizeof(leds));
  ws2812_init();
  clearLEDs();
  delay(100);

  Serial.println("[WS2812B] Self-test: R -> G -> B");
  setAllLEDs(50, 0, 0);  ws2812_show(); delay(300);
  setAllLEDs(0, 50, 0);  ws2812_show(); delay(300);
  setAllLEDs(0, 0, 50);  ws2812_show(); delay(300);
  clearLEDs();

  battery_adc_init();
  batteryVoltage = readBatteryVoltage();
  Serial.printf("[Battery] Initial voltage: %.2fV\n", batteryVoltage);

  oled.begin();
  drawStaticUI();
  Serial.println("[OK] System ready.");

  calibStartTimeMs = millis();
  motorsStop();
  Serial.println("[OK] Motors stopped. Waiting for UART cmd_vel frames.");
  //demoTransition(DEMO_FORWARD, millis());  // 若需开机自动演示，取消此行注释

  pidTicker.attach_ms(10, pid_update);
  Serial.println("[OK] PID Ticker started at 100Hz.");
}

void loop() {
  static unsigned long lastUpdateMs = 0;
  static unsigned long lastWs2812Ms = 0;
  static unsigned long lastBatMs = 0;
  static unsigned long lastLedToggleMs = 0;
  static unsigned long lastStatusMs = 0;
  static bool ledIndicatorState = false;
  static unsigned long lastButtonPressMs = 0;
  static bool lastButtonState = HIGH;
  unsigned long now = millis();

  pollPiSerial();
  checkCommandTimeout(now);

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendRobotStatus();
  }

  // ── 按键检测 ──
  bool btnNow = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && btnNow == LOW
      && (now - lastButtonPressMs > BUTTON_DEBOUNCE_MS)) {
    lastButtonPressMs = now;
    Serial.println("[Button] Pressed!");
  }
  lastButtonState = btnNow;

  // ── OLED + 编码器更新 (每50ms) ──
  if (now - lastUpdateMs >= 50) {
    lastUpdateMs = now;

    noInterrupts();
    int32_t c1 = enc1Count;
    int32_t c2 = enc2Count;
    interrupts();

    float a1 = calcAngle(c1);
    float a2 = calcAngle(c2);

    updateDisplay(c1, a1, c2, a2, batteryVoltage);

    static unsigned long lastSerialMs = 0;
    if (now - lastSerialMs >= 500) {
      lastSerialMs = now;

      // PID 系统已统一使用 RPM，直接打印变量
      Serial.printf("M1[Spd:%.2f RPM Tgt:%.2f RPM PWM:%d Dir:%d] M2[Spd:%.2f RPM Tgt:%.2f RPM PWM:%d Dir:%d] BAT:%.1fV St:%d\n",
                     speedM1, pidM1.target, pwmOutM1, motor1EncDir,
                     speedM2, pidM2.target, pwmOutM2, motor2EncDir,
                     batteryVoltage, (int)demoState);
    }
  }

  // ── WS2812B 更新 (每40ms) ──
  if (now - lastWs2812Ms >= 40) {
    lastWs2812Ms = now;
    if (batteryVoltage > 0.5f && batteryVoltage < LOW_BATTERY_THRESHOLD) {
      ws2812_low_battery_flash();
    } else {
      ws2812_rainbow_update();
    }
  }

  // ── LED 指示灯缓慢闪烁 (每800ms) ──
  if (now - lastLedToggleMs >= 800) {
    lastLedToggleMs = now;
    ledIndicatorState = !ledIndicatorState;
    digitalWrite(LED_PIN_1, ledIndicatorState ? HIGH : LOW);
    digitalWrite(LED_PIN_2, ledIndicatorState ? HIGH : LOW);
    digitalWrite(LED_PIN_3, ledIndicatorState ? HIGH : LOW);
  }

  // ── 电池电压采样 (每500ms) ──
  if (now - lastBatMs >= 500) {
    lastBatMs = now;
    float rawV = readBatteryVoltage();
    batteryVoltage = batteryVoltage * (1.0f - BATTERY_FILTER_ALPHA)
                   + rawV * BATTERY_FILTER_ALPHA;
  }
}
