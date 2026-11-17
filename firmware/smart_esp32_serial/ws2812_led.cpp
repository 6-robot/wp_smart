#include "ws2812_led.h"
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

namespace {
// Construction allocates the buffer but does not drive GPIO45 before setup().
Adafruit_NeoPixel strip(WS2812_LED_COUNT, WS2812_DATA_PIN, NEO_GRB + NEO_KHZ800);
uint32_t colors[WS2812_LED_COUNT] = {};
uint8_t brightness = WS2812_DEFAULT_BRIGHTNESS;
bool ready = false;
bool dirty = false;
enum class MotionEffect { Breath, Standby, Forward, Left, Right, Reverse };
MotionEffect effect = MotionEffect::Breath;
bool effectStarted = false;
uint32_t effectStartMs = 0, lastEffectFrameMs = 0;
constexpr uint8_t breatheColors[][3] = {
  {255, 0, 0}, {255, 255, 0}, {0, 255, 0},
  {0, 255, 255}, {0, 0, 255}, {255, 0, 255},
};
constexpr uint8_t breatheColorCount = sizeof(breatheColors) / sizeof(breatheColors[0]);

MotionEffect selectEffect(float linear, float angular, bool standby) {
  if (standby) return MotionEffect::Standby;
  if (!isfinite(linear) || !isfinite(angular)) return MotionEffect::Breath;
  if (angular > WS2812_ANGULAR_ZERO_RADPS) return MotionEffect::Left;
  if (angular < -WS2812_ANGULAR_ZERO_RADPS) return MotionEffect::Right;
  if (linear > WS2812_LINEAR_ZERO_MPS) return MotionEffect::Forward;
  if (linear < -WS2812_LINEAR_ZERO_MPS) return MotionEffect::Reverse;
  return MotionEffect::Breath;
}
}  // namespace

bool ws2812_init() {
  // Preload LOW before begin() enables output. Keep the external boot pulldown.
  digitalWrite(WS2812_DATA_PIN, LOW);
  ready = strip.numPixels() == WS2812_LED_COUNT && strip.begin();
  if (!ready) return false;
  brightness = WS2812_DEFAULT_BRIGHTNESS;
  strip.setBrightness(brightness);
  for (uint8_t i = 0; i < WS2812_LED_COUNT; ++i) colors[i] = 0;
  strip.clear();
  strip.show();  // Start with an all-off frame before the idle breathing effect.
  dirty = false;
  effectStarted = false;
  return true;
}

void ws2812_set_all(uint8_t r, uint8_t g, uint8_t b) {
  for (uint8_t i = 0; i < WS2812_LED_COUNT; ++i) ws2812_set_pixel(i, r, g, b);
}

void ws2812_set_pixel(int index, uint8_t r, uint8_t g, uint8_t b) {
  if (!ready || index < 0 || index >= WS2812_LED_COUNT) return;
  uint32_t color = strip.Color(r, g, b);  // NeoPixel handles the GRB wire order.
  if (colors[index] == color) return;
  colors[index] = color;
  dirty = true;
}

void ws2812_set_brightness(uint8_t value) {
  if (!ready || brightness == value) return;
  brightness = value;
  dirty = true;
}

void ws2812_clear() {
  ws2812_set_all(0, 0, 0);
}

void ws2812_show() {
  if (!ready || !dirty || !strip.canShow()) return;
  strip.setBrightness(brightness);
  // Redraw from unscaled colors so brightness 0 -> 255 restores the colors.
  for (uint8_t i = 0; i < WS2812_LED_COUNT; ++i) strip.setPixelColor(i, colors[i]);
  strip.show();
  dirty = false;
}

void ws2812_update_motion(float linear, float angular, uint32_t now_ms, bool standby) {
  if (!ready) return;
  MotionEffect next = selectEffect(linear, angular, standby);
  if (!effectStarted) {
    effectStarted = true;
    effect = next;
    effectStartMs = now_ms;
    lastEffectFrameMs = now_ms - WS2812_FRAME_MS;
  } else if (next != effect) {
    effect = next;
    effectStartMs = now_ms;
  }
  if (uint32_t(now_ms-lastEffectFrameMs) < WS2812_FRAME_MS) return;
  lastEffectFrameMs = now_ms;
  uint32_t elapsed = uint32_t(now_ms-effectStartMs);
  ws2812_set_brightness(WS2812_DEFAULT_BRIGHTNESS);
  switch (effect) {
    case MotionEffect::Breath:
    case MotionEffect::Standby: {
      float phase = float(elapsed % WS2812_BREATHE_MS) / WS2812_BREATHE_MS;
      uint8_t level = uint8_t(lroundf(127.5f * (1.0f - cosf(6.28318530718f * phase))));
      if (effect == MotionEffect::Breath) {
        ws2812_set_all(0, 0, level);
      } else {
        // Crossfade adjacent colors and keep a visible floor throughout the breath.
        uint8_t index = (elapsed / WS2812_BREATHE_MS) % breatheColorCount;
        const uint8_t* from = breatheColors[index];
        const uint8_t* to = breatheColors[(index + 1) % breatheColorCount];
        float blend = 0.5f * (1.0f - cosf(3.14159265359f * phase));
        uint8_t floorLevel = WS2812_STANDBY_MIN_LEVEL;
        float intensity = floorLevel + (255.0f - floorLevel) * level / 255.0f;
        uint8_t rgb[3];
        for (uint8_t channel = 0; channel < 3; ++channel) {
          float color = from[channel] + (float(to[channel]) - from[channel]) * blend;
          rgb[channel] = uint8_t(lroundf(color * intensity / 255.0f));
        }
        ws2812_set_all(rgb[0], rgb[1], rgb[2]);
      }
      break;
    }
    case MotionEffect::Forward:
      ws2812_set_all(0, 255, 0);
      break;
    case MotionEffect::Left:
    case MotionEffect::Right: {
      uint8_t step = (elapsed / WS2812_CHASE_STEP_MS) % (WS2812_LED_COUNT + 1);
      bool descending = (effect == MotionEffect::Left) != WS2812_REVERSE_ORDER;
      bool reversing = linear < -WS2812_LINEAR_ZERO_MPS;
      for (uint8_t index = 0; index < WS2812_LED_COUNT; ++index) {
        uint8_t rank = descending ? WS2812_LED_COUNT - 1 - index : index;
        // The seventh step is dark; unchanged steps leave the cache clean.
        uint8_t level = step < WS2812_LED_COUNT && rank <= step ? 255 : 0;
        ws2812_set_pixel(index, level, reversing ? 0 : level, 0);
      }
      break;
    }
    case MotionEffect::Reverse:
      ws2812_set_all((elapsed % WS2812_REVERSE_PERIOD_MS) <
        WS2812_REVERSE_PERIOD_MS / 2 ? 255 : 0, 0, 0);
      break;
  }
}
