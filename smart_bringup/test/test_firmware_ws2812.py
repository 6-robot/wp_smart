"""Exercise the real NeoPixel color encoder with a native ESP32 transport stub."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.fixture(scope='module')
def native_ws2812(tmp_path_factory):
    return _build_native(tmp_path_factory.mktemp('ws2812'), False)


@pytest.fixture(scope='module')
def reversed_ws2812(tmp_path_factory):
    return _build_native(tmp_path_factory.mktemp('ws2812_reversed'), True)


def _build_native(directory, reverse_order):
    compiler = shutil.which('g++')
    library = Path(os.environ.get(
        'ARDUINO_NEOPIXEL_DIR', Path.home() / 'Arduino/libraries/Adafruit_NeoPixel'))
    if compiler is None or not (library / 'Adafruit_NeoPixel.cpp').is_file():
        pytest.skip('requires g++ and Adafruit NeoPixel; see docs/ws2812_led.md')
    firmware = Path(__file__).resolve().parents[2] / 'firmware/smart_esp32_serial'
    header = (firmware / 'ws2812_led.h').read_text()
    if reverse_order:
        header = header.replace('WS2812_REVERSE_ORDER = false', 'WS2812_REVERSE_ORDER = true')
    (directory / 'ws2812_led.h').write_text(header)
    shutil.copyfile(firmware / 'ws2812_led.cpp', directory / 'ws2812_led.cpp')
    (directory / 'Arduino.h').write_text(r'''
#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctype.h>
#define PROGMEM
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#define ESP_IDF_VERSION_VAL(a,b,c) (((a)<<16)|((b)<<8)|(c))
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(4,4,7)
#define INPUT 0
#define OUTPUT 1
#define LOW 0
using boolean = bool;
uint32_t micros();
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
''')
    program = directory / 'ws2812_test.cpp'
    program.write_text(r'''
#include <cassert>
#include <climits>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include "ws2812_led.h"
uint32_t clock_us = 1000;
bool recording = true;
unsigned gpio_calls = 0;
std::vector<std::vector<uint8_t>> frames;
uint32_t micros() { return clock_us++; }
void pinMode(int pin, int mode) {
  if (recording) { assert(pin==45 && mode==1); ++gpio_calls; }
}
void digitalWrite(int pin, int value) {
  if (recording) { assert(pin==45 && value==0); ++gpio_calls; }
}
extern "C" void espShow(uint16_t pin, uint8_t* pixels, uint32_t bytes, uint8_t khz800) {
  if (!recording) return;
  assert(pin==45 && bytes==18 && khz800);
  frames.emplace_back(pixels, pixels+bytes);
}
void flush() { clock_us+=1000; ws2812_show(); }
void expect_all(uint8_t r, uint8_t g, uint8_t b) {
  assert(frames.back().size()==18);
  for (int i=0;i<6;++i) {
    assert(frames.back()[3*i]==g);
    assert(frames.back()[3*i+1]==r);
    assert(frames.back()[3*i+2]==b);
  }
}
void animate(float linear, float angular, uint32_t now, bool standby=false) {
  ws2812_update_motion(linear,angular,now,standby);flush();
}
void expect_chase(bool left, int count, bool reversing=false) {
  for (int i=0;i<6;++i) {
    bool descending=left!=WS2812_REVERSE_ORDER;
    bool lit=(descending?5-i:i)<count;
    assert(frames.back()[3*i]==(lit&&!reversing?32:0));
    assert(frames.back()[3*i+1]==(lit?32:0));
    assert(frames.back()[3*i+2]==0);
  }
}
int main(int argc, char** argv) {
  assert(argc>=2 && gpio_calls==0);  // Global construction never drives the strap pin.
  ws2812_set_all(255,255,255);ws2812_show();
  assert(frames.empty());  // Calls before initialization are harmless.
  assert(ws2812_init());
  assert(frames.size()==1 && gpio_calls==3);
  expect_all(0,0,0);
  const std::string mode=argv[1];
  if (mode=="color") {
    assert(argc==5);
    uint8_t r=std::stoi(argv[2]),g=std::stoi(argv[3]),b=std::stoi(argv[4]);
    ws2812_set_brightness(255);ws2812_set_all(r,g,b);
    assert(frames.size()==1);  // Setters do not transmit.
    flush();expect_all(r,g,b);
    ws2812_clear();flush();expect_all(0,0,0);
  } else if (mode=="bounds") {
    for (int index : {-1,6,255,INT_MIN,INT_MAX}) ws2812_set_pixel(index,255,255,255);
    flush();assert(frames.size()==1);
    ws2812_set_brightness(255);
    ws2812_set_pixel(0,1,2,3);ws2812_set_pixel(5,4,5,6);
    flush();assert(frames.size()==2);
    const auto& frame=frames.back();
    assert(frame[0]==2 && frame[1]==1 && frame[2]==3);
    for (int i=3;i<15;++i) assert(frame[i]==0);
    assert(frame[15]==5 && frame[16]==4 && frame[17]==6);
  } else if (mode=="brightness") {
    ws2812_set_all(255,255,255);flush();expect_all(32,32,32);
    ws2812_set_brightness(0);flush();expect_all(0,0,0);
    ws2812_set_brightness(255);flush();expect_all(255,255,255);
    ws2812_set_all(17,83,149);flush();expect_all(17,83,149);
    for (int i=0;i<20;++i) {
      ws2812_set_brightness(1);flush();
      ws2812_set_brightness(255);flush();expect_all(17,83,149);
    }
    ws2812_clear();flush();
    ws2812_set_brightness(32);flush();expect_all(0,0,0);
  } else if (mode=="dirty") {
    for (int i=0;i<1000;++i) flush();
    assert(frames.size()==1);  // Idle loop does not repeatedly refresh.
    ws2812_set_all(255,0,0);ws2812_set_all(0,255,0);ws2812_set_all(0,0,255);
    ws2812_set_pixel(3,255,255,0);
    assert(frames.size()==1);
    flush();assert(frames.size()==2);  // Batch changes into one frame.
    ws2812_set_all(0,0,255);flush();
    unsigned count=frames.size();
    ws2812_set_all(0,0,255);ws2812_set_brightness(32);flush();
    assert(frames.size()==count);
    ws2812_set_all(0,255,0);flush();count=frames.size();
    ws2812_set_all(255,0,0);ws2812_show();
    assert(frames.size()==count);  // Defer during the latch interval.
    flush();assert(frames.size()==count+1);expect_all(32,0,0);
  } else if (mode=="select") {
    assert(argc==5);
    float linear=std::stof(argv[2]),angular=std::stof(argv[3]);
    std::string expected=argv[4];
    animate(linear,angular,1000);
    if (expected=="forward") expect_all(0,32,0);
    else if (expected=="reverse") expect_all(32,0,0);
    else if (expected=="left") expect_chase(true,1);
    else if (expected=="right") expect_chase(false,1);
    else if (expected=="reverse_left") expect_chase(true,1,true);
    else if (expected=="reverse_right") expect_chase(false,1,true);
    else {
      expect_all(0,0,0);
      animate(linear,angular,2000);expect_all(0,0,32);
    }
  } else if (mode=="breathe") {
    int previous=0;
    for (uint32_t elapsed=0;elapsed<=2000;elapsed+=20) {
      animate(0,0,1000+elapsed);
      int blue=frames.back()[2];
      expect_all(0,0,blue);assert(blue<=32);
      if (elapsed<=1000) assert(blue>=previous);
      else assert(blue<=previous);
      if (elapsed==0 || elapsed==2000) assert(blue==0);
      if (elapsed==1000) assert(blue==32);
      previous=blue;
    }
    assert(frames.size()<101);  // Quantized identical frames are not retransmitted.
  } else if (mode=="left" || mode=="right" || mode=="reverse_left" || mode=="reverse_right") {
    static_assert(WS2812_CHASE_STEP_MS==100,"faster turn chase");
    bool left=mode=="left" || mode=="reverse_left";
    bool reversing=mode=="reverse_left" || mode=="reverse_right";
    float linear=reversing?-0.03f:0;
    float angular=left?0.2f:-0.2f;
    for (int step=0;step<7;++step) {
      animate(linear,angular,1000+step*100);
      expect_chase(left,step<6?step+1:0,reversing);
      size_t count=frames.size();
      animate(linear,angular,1020+step*100);
      assert(frames.size()==count);  // No repeated sends within a chase step.
    }
    animate(linear,angular,1700);expect_chase(left,1,reversing);
  } else if (mode=="turn_color_transition") {
    animate(0,0.2f,1000);expect_chase(true,1);
    animate(-0.03f,0.2f,1100);expect_chase(true,2,true);
    animate(0.03f,0.2f,1200);expect_chase(true,3);
    animate(-0.03f,-0.2f,1300);expect_chase(false,1,true);
    animate(-0.03f,0,1400);expect_all(32,0,0);
    animate(0,0,1500);expect_all(0,0,0);
  } else if (mode=="reverse") {
    for (uint32_t elapsed : {0,980,1000,1980,2000,2980,3000,3980,4000}) {
      animate(-0.03f,0,1000+elapsed);
      expect_all(elapsed%2000<1000?32:0,0,0);
    }
  } else if (mode=="keepalive") {
    animate(0,0.2f,1000);expect_chase(true,1);
    animate(0.03f,0.3f,1500);expect_chase(true,6);  // Same mode, different speed.
    animate(0,0.2f,1600);expect_chase(true,0);
    animate(0,-0.2f,2400);expect_chase(false,1);  // Direction change resets phase.
    animate(0,0,2600);expect_all(0,0,0);
    animate(0,0,3600);expect_all(0,0,32);  // Zero keepalive does not reset breathing.
    animate(0.03f,0,3620);expect_all(0,32,0);
    animate(0,0.2f,3640);expect_chase(true,1);
  } else if (mode=="rate") {
    animate(0.03f,0,1000);expect_all(0,32,0);
    size_t count=frames.size();
    animate(0,-0.2f,1001);animate(0,-0.2f,1019);
    assert(frames.size()==count);
    animate(0,-0.2f,1020);expect_chase(false,1);
    assert(frames.size()==count+1);
  } else if (mode=="wrap") {
    uint32_t start=0xffffff00U;
    animate(0,0.2f,start);expect_chase(true,1);
    animate(0,0.2f,start+500);expect_chase(true,6);
    animate(0,0.2f,start+600);expect_chase(true,0);
    animate(0,0.2f,start+700);expect_chase(true,1);
  } else if (mode=="standby") {
    const uint8_t palette[][3]={
      {8,0,0},{8,8,0},{0,8,0},{0,8,8},{0,0,8},{8,0,8},
    };
    const uint8_t midpoints[][3]={
      {32,16,0},{16,32,0},{0,32,16},{0,16,32},{16,0,32},{32,0,16},
    };
    for (int cycle=0;cycle<7;++cycle) {
      animate(0,0,1000+cycle*2000,true);
      const auto& color=palette[cycle%6];expect_all(color[0],color[1],color[2]);
      animate(0,0,2000+cycle*2000,true);
      const auto& midpoint=midpoints[cycle%6];
      expect_all(midpoint[0],midpoint[1],midpoint[2]);
    }
  } else if (mode=="standby_no_blackout") {
    std::vector<uint8_t> previous;
    for (uint32_t now=1000;now<=13000;now+=20) {
      animate(0,0,now,true);
      const auto& current=frames.back();
      for (int i=0;i<6;++i) {
        int maximum=0;
        for (int channel=0;channel<3;++channel) {
          int value=current[3*i+channel];
          assert(value<=32);
          if (value>maximum) maximum=value;
          if (!previous.empty()) assert(std::abs(value-int(previous[3*i+channel]))<=2);
        }
        assert(maximum>=8);  // Includes all six color boundaries and palette wrap.
      }
      previous=current;
    }
    expect_all(8,0,0);
  } else if (mode=="standby_transition") {
    animate(0,0,1000,true);expect_all(8,0,0);
    animate(0,0,2000,true);expect_all(32,16,0);
    animate(0,0,2020,false);expect_all(0,0,0);  // Valid zero -> blue idle.
    animate(0,0,3020,false);expect_all(0,0,32);
    animate(0,0,3040,true);expect_all(8,0,0);  // Lease expires -> multicolor.
    animate(0,0,4040,true);expect_all(32,16,0);
    animate(0.03f,0,4060,false);expect_all(0,32,0);
    animate(0,0,4080,true);expect_all(8,0,0);
  } else if (mode=="standby_continuity") {
    for (uint32_t now=1000;now<=4000;now+=20) animate(0,0,now,true);
    expect_all(16,32,0);  // Repeated standby updates do not reset the palette.
  } else if (mode=="standby_wrap") {
    uint32_t start=0xffffff00U;
    animate(0,0,start,true);expect_all(8,0,0);
    animate(0,0,start+1000,true);expect_all(32,16,0);
    animate(0,0,start+2000,true);expect_all(8,8,0);
    animate(0,0,start+3000,true);expect_all(16,32,0);
  } else if (mode=="invalid") {
    animate(std::numeric_limits<float>::quiet_NaN(),0,1000);expect_all(0,0,0);
    animate(0,std::numeric_limits<float>::infinity(),2000);expect_all(0,0,32);
  } else {
    assert(false);
  }
  recording=false;  // Do not record the real library's shutdown/destructor calls.
}
''')
    executable = directory / 'ws2812_test'
    result = subprocess.run([
        compiler, '-std=c++17', '-Wall', '-Wextra', '-DARDUINO=10819', '-DESP32',
        '-DARDUINO_ARCH_ESP32', '-I', str(directory), '-I', str(firmware),
        '-I', str(library), str(program), str(directory / 'ws2812_led.cpp'),
        str(library / 'Adafruit_NeoPixel.cpp'), '-o', str(executable),
    ], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    return executable


def _run(executable, *arguments):
    result = subprocess.run([str(executable), *map(str, arguments)],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('color', [
    (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 255, 255), (0, 0, 0),
])
def test_required_colors_and_clear_use_grb(native_ws2812, color):
    _run(native_ws2812, 'color', *color)


@pytest.mark.parametrize('mode', ['bounds', 'brightness', 'dirty'])
def test_pixel_bounds_brightness_and_refresh(native_ws2812, mode):
    _run(native_ws2812, mode)


@pytest.mark.parametrize('linear,angular,expected', [
    (0, 0, 'breath'), (0.03, 0, 'forward'), (-0.03, 0, 'reverse'),
    (0, 0.2, 'left'), (0, -0.2, 'right'),
    (0.03, 0.2, 'left'), (0.03, -0.2, 'right'),
    (-0.03, 0.2, 'reverse_left'), (-0.03, -0.2, 'reverse_right'),
    (-0.001, 0.2, 'left'), (-0.001, -0.2, 'right'),
    (-0.0011, 0.2, 'reverse_left'), (-0.0011, -0.2, 'reverse_right'),
    (0.001, 0.01, 'breath'), (-0.001, -0.01, 'breath'),
    (0.0011, 0, 'forward'), (-0.0011, 0, 'reverse'),
    (0, 0.0101, 'left'), (0, -0.0101, 'right'),
])
def test_motion_selection_and_thresholds(native_ws2812, linear, angular, expected):
    _run(native_ws2812, 'select', linear, angular, expected)


@pytest.mark.parametrize('mode', [
    'breathe', 'left', 'right', 'reverse', 'keepalive', 'rate', 'wrap', 'invalid',
    'standby', 'standby_transition', 'standby_continuity', 'standby_wrap',
    'standby_no_blackout',
    'reverse_left', 'reverse_right', 'turn_color_transition',
])
def test_motion_animation_timing(native_ws2812, mode):
    _run(native_ws2812, mode)


@pytest.mark.parametrize('direction', ['left', 'right', 'reverse_left', 'reverse_right'])
def test_reversed_physical_order(reversed_ws2812, direction):
    _run(reversed_ws2812, direction)
