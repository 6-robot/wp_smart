"""Native OLED regressions; these compile helpers, never upload to hardware."""
from pathlib import Path
import shutil
import subprocess

import pytest


def _run_native(tmp_path, program):
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('native firmware regression requires g++')
    firmware = Path(__file__).resolve().parents[2] / 'firmware/smart_esp32_serial'
    source = tmp_path / 'display_test.cpp'
    source.write_text(program)
    executable = tmp_path / 'display_test'
    result = subprocess.run([
        compiler, '-std=c++17', '-Wall', '-Wextra', '-I', str(tmp_path), '-I', str(firmware),
        str(source), '-o', str(executable),
    ], capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stderr
    result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr


def test_front_sector_missing_returns_and_expiry(tmp_path):
    _run_native(tmp_path, r'''
#include <cassert>
#include "display_data.h"
struct Point { uint16_t angle_cdeg, range_mm; };
int main() {
  FrontDistance front;
  assert(!front.valid(0));
  Point points[] = {{0,250},{17499,250},{17500,2000},{18000,1200},
                    {18500,1500},{18501,250},{18000,0},{18000,239},{18000,10001}};
  front.update(points, 9, 100);
  assert(front.range_mm==1200 && front.valid(1099) && !front.valid(1100));
  Point empty[] = {{0,500},{18000,0}};
  front.update(empty, 2, 200);
  assert(!front.valid(200));
  front.update(points, 9, 0xfffffff0U);
  assert(front.valid(0x20U) && !front.valid(0x3d8U));
}
''')


def test_oled_flush_is_bounded_and_stops_after_i2c_errors(tmp_path):
    (tmp_path / 'Arduino.h').write_text(r'''
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#define PROGMEM
#define pgm_read_byte(p) (*(p))
''')
    (tmp_path / 'Wire.h').write_text(r'''
#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
struct FakeWire {
  bool fail=false;
  std::vector<uint8_t> current;
  std::vector<std::vector<uint8_t>> transactions;
  void beginTransmission(int address) { current.clear(); if(address!=0x3c) fail=true; }
  void write(uint8_t value) { current.push_back(value); }
  void write(const uint8_t* bytes, size_t count) {
    current.insert(current.end(),bytes,bytes+count);
  }
  int endTransmission() { transactions.push_back(current); return fail?2:0; }
};
inline FakeWire Wire;
''')
    _run_native(tmp_path, r'''
#include <cassert>
#include "status_display.h"
int main() {
  StatusDisplay display;
  assert(display.begin() && !display.pending());
  assert(Wire.transactions.size()==1 && Wire.transactions[0].size()<=32);
  display.clear();
  display.drawString(0,0,"WP-SMART 20261115");
  display.drawString(0,10,"BAT:12.60V");
  display.drawString(0,30,"L ENC:-2147483648");
  display.drawString(0,40,"R ENC:2147483647");
  display.drawString(0,50,"IMU:OK    LIDAR:OK");
  display.drawString(127,63,"overflow");
  display.present();
  size_t lit=0;
  for (int chunk=0;chunk<64;++chunk) {
    assert(display.pending());
    size_t before=Wire.transactions.size();
    display.flushChunk();
    assert(Wire.transactions.size()==before+2);
    const auto& address=Wire.transactions[before];
    const auto& data=Wire.transactions[before+1];
    assert(address.size()==4 && address[1]==0xb0+chunk/8);
    assert(data.size()==17 && data[0]==0x40);
    for(size_t i=1;i<data.size();++i) if(data[i]) ++lit;
  }
  assert(!display.pending() && lit>0);
  size_t before=Wire.transactions.size();
  display.flushChunk();
  assert(Wire.transactions.size()==before);
  display.present();
  Wire.fail=true;
  display.flushChunk();
  assert(!display.ready() && !display.pending());
  before=Wire.transactions.size();
  display.flushChunk();
  assert(Wire.transactions.size()==before);
  StatusDisplay missing;
  assert(!missing.begin() && !missing.ready());
}
''')
