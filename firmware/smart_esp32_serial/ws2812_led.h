#pragma once
#include <stdint.h>

constexpr uint8_t WS2812_LED_COUNT = 6;
constexpr uint8_t WS2812_DATA_PIN = 45;
constexpr uint8_t WS2812_DEFAULT_BRIGHTNESS = 32;
// Physical order is provisionally 0=vehicle left, 5=vehicle right.
constexpr bool WS2812_REVERSE_ORDER = false;
constexpr uint32_t WS2812_FRAME_MS = 20;
constexpr uint32_t WS2812_BREATHE_MS = 2000;
constexpr uint8_t WS2812_STANDBY_MIN_LEVEL = 64;  // ~8/255 after overall brightness.
constexpr uint32_t WS2812_CHASE_STEP_MS = 100;
constexpr uint32_t WS2812_REVERSE_PERIOD_MS = 2000;
constexpr float WS2812_LINEAR_ZERO_MPS = 0.001f;
constexpr float WS2812_ANGULAR_ZERO_RADPS = 0.01f;

// Call only from setup()/loop(), never from an ISR, timer or the PID task.
// Initialization transmits an all-off frame. False means initialization failed.
bool ws2812_init();

// Setters cache changes; ws2812_show() sends them together from loop().
void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_pixel(int index, uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_brightness(uint8_t brightness);
void ws2812_clear();

// Sends only changed data, and defers until the NeoPixel latch interval passes.
void ws2812_show();

// Main-loop animation: effective command speeds; zero means normal blue idle.
// standby=true selects multicolor breathing while waiting or after expiry.
// Turn indications take priority over forward/reverse. Rendering is <=50 Hz.
void ws2812_update_motion(float linear, float angular, uint32_t now_ms, bool standby = false);
