"""Verify the serial reference firmware's encoder conversion independently."""
from pathlib import Path
import re
import shutil
import subprocess

import pytest


def _function(source, name):
    match = re.search(r'^[\w *&]+\b' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    start = source.index('{', match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


def test_serial_firmware_encoder_conversion_at_64_to_one(tmp_path):
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('native firmware regression requires g++')
    source = (Path(__file__).resolve().parents[1] / 'doc' / 'ros2_esp32.ino').read_text()
    macros = '\n'.join(re.search(r'^#define ' + name + r'\s+[^\n]+', source, re.M)[0]
                       for name in ('ENCODER_PPR', 'COUNTS_PER_REV', 'REDUCTION_RATIO',
                                    'SPEED_FILTER_ALPHA'))
    pid = _function(source, 'pid_update')
    encoder_step = pid[pid.index('  int16_t pcnt1'):pid.index('  // ══ 电机控制')]
    program = tmp_path / 'serial_firmware_gearing.cpp'
    program.write_text(r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#define PCNT_UNIT_0 0
#define PCNT_UNIT_1 1
void pcnt_get_counter_value(int unit,int16_t *value){*value=unit==0?1024:-1024;}
int32_t enc1Count=0,enc2Count=0;
int motor1EncDir=1,motor2EncDir=1;
float speedM1=0,speedM2=0;
struct PID {float target=0;}; PID pidM1,pidM2;
''' + macros + '\nvoid sampleEncoders(){\n' + encoder_step + r'''
}
int main(){
  static_assert(REDUCTION_RATIO==64, "64:1 hardware");
  static_assert(COUNTS_PER_REV*REDUCTION_RATIO==262144, "wheel encoder scale");
  sampleEncoders();
  assert(enc1Count==1024 && enc2Count==-1024);
  assert(std::fabs(speedM1/SPEED_FILTER_ALPHA-23.4375f)<1e-5f);
  assert(std::fabs(speedM2/SPEED_FILTER_ALPHA+23.4375f)<1e-5f);
}
''')
    executable = tmp_path / 'serial_firmware_gearing'
    compiled = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', str(program),
                               '-o', str(executable)], capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr
    run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=5)
    assert run.returncode == 0, run.stderr


def test_new_serial_motion_lease_and_clock_wrap(tmp_path):
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('native firmware regression requires g++')
    firmware = Path(__file__).resolve().parents[2] / 'firmware' / 'smart_esp32_serial'
    source = (firmware / 'smart_esp32_serial.ino').read_text()
    timeout_macro = re.search(r'^#define CMD_TIMEOUT_MS\s+[^\n]+', source, re.M)[0]
    program = tmp_path / 'motion_guard.cpp'
    program.write_text(r'''
#include <cassert>
#include "motion_guard.h"
''' + timeout_macro + r'''
int main() {
  static_assert(CMD_TIMEOUT_MS==3000, "three-second firmware command lease");
  MotionGuard guard;
  guard.expire(4000, CMD_TIMEOUT_MS);
  assert(!guard.valid && guard.left == 0 && !guard.timed_out);
  guard.accept(10, -10, 100);
  guard.expire(3099, CMD_TIMEOUT_MS);
  assert(guard.valid && guard.left == 10);
  guard.expire(3100, CMD_TIMEOUT_MS);
  assert(!guard.valid && guard.left == 0 && guard.right == 0 && guard.timed_out);
  guard.accept(1, -1, 0xfffffff0U);
  guard.expire(0xba7U, CMD_TIMEOUT_MS);
  assert(guard.valid);
  guard.expire(0xba8U, CMD_TIMEOUT_MS);
  assert(!guard.valid && guard.timed_out);
  guard.accept(0, 0, 4000);
  guard.expire(7000, CMD_TIMEOUT_MS);
  assert(!guard.valid && !guard.timed_out);
  guard.accept(10, 10, 8000);
  guard.stop();
  assert(!guard.valid && guard.left == 0 && guard.right == 0);
  guard.accept(10, -10, 9000);
  guard.expire(11999, CMD_TIMEOUT_MS);
  assert(guard.valid);
  guard.accept(20, -20, 11999);  // Fresh valid commands restart the full lease.
  guard.expire(14998, CMD_TIMEOUT_MS);
  assert(guard.valid && guard.left == 20);
  guard.expire(14999, CMD_TIMEOUT_MS);
  assert(!guard.valid && guard.timed_out);
}
''')
    executable = tmp_path / 'motion_guard'
    result = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra',
                             '-I', str(firmware), str(program), '-o', str(executable)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr


def test_new_serial_pid_scale_and_stop_without_loop(tmp_path):
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('native firmware regression requires g++')
    firmware = Path(__file__).resolve().parents[2] / 'firmware' / 'smart_esp32_serial'
    source = (firmware / 'smart_esp32_serial.ino').read_text()
    macros = '\n'.join(re.search(r'^#define ' + name + r'\s+[^\n]+', source, re.M)[0]
                       for name in ('ENCODER_PPR', 'COUNTS_PER_REV', 'REDUCTION_RATIO',
                                    'CMD_TIMEOUT_MS'))
    program = tmp_path / 'serial_pid.cpp'
    program.write_text(r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "motion_guard.h"
#define PCNT_UNIT_0 0
#define PCNT_UNIT_1 1
#define portENTER_CRITICAL(x) (void)(x)
#define portEXIT_CRITICAL(x) (void)(x)
int commandMux=0, telemetryMux=0;
uint32_t clock_ms=100;
uint32_t millis(){return clock_ms;}
int16_t hardware_counts[2]={1024,-1024};
int duty[4]={};
void pcnt_get_counter_value(int unit,int16_t *value){*value=hardware_counts[unit];}
void ledcWrite(int channel,int value){duty[channel]=value;}
template<class T> T constrain(T v,T lo,T hi){return v<lo?lo:(v>hi?hi:v);}
MotionGuard motion;
struct MotorState {
  float rpm[2]={}, targets[2]={};
  int32_t encoders[2]={}; int16_t pwm[2]={};
} motorState;
''' + macros + '\n' + _function(source, 'pid_update') + r'''
int main(){
  static_assert(COUNTS_PER_REV*REDUCTION_RATIO==262144, "wheel count scale");
  pid_update();
  assert(motorState.encoders[0]==1024 && motorState.encoders[1]==-1024);
  assert(std::fabs(motorState.rpm[0]/0.6f-23.4375f)<1e-5f);
  assert(motorState.pwm[0]==0 && motorState.pwm[1]==0);
  motion.accept(30,-30,100);
  clock_ms=110;
  pid_update();
  assert(motorState.pwm[0]>0 && motorState.pwm[1]<0);
  // Do not call loop() or any serial routine: the PID task must stop on its own.
  clock_ms=3099;
  pid_update();
  assert(motion.valid && motorState.targets[0]==30 && motorState.targets[1]==-30);
  clock_ms=3100;
  pid_update();
  assert(!motion.valid && motion.timed_out);
  assert(motorState.targets[0]==0 && motorState.targets[1]==0);
  assert(motorState.pwm[0]==0 && motorState.pwm[1]==0);
  for(int value:duty) assert(value==255);  // DRV8870 braking.
}
''')
    executable = tmp_path / 'serial_pid'
    result = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra',
                             '-I', str(firmware), str(program), '-o', str(executable)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr


def test_new_serial_command_parser_rejects_bad_commands(tmp_path):
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('native firmware regression requires g++')
    firmware = Path(__file__).resolve().parents[2] / 'firmware' / 'smart_esp32_serial'
    source = (firmware / 'smart_esp32_serial.ino').read_text()
    macros = '\n'.join(re.search(r'^#define ' + name + r'\s+[^\n]+', source, re.M)[0]
                       for name in ('CMD_TIMEOUT_MS', 'MAX_LINEAR_MPS', 'MAX_ANGULAR_RADPS',
                                    'WHEEL_DIAMETER_M', 'WHEEL_BASE_M'))
    program = tmp_path / 'serial_commands.cpp'
    program.write_text(r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>
#include "motion_guard.h"
#define portENTER_CRITICAL(x) (void)(x)
#define portEXIT_CRITICAL(x) (void)(x)
#define PI 3.14159265358979323846
using std::isfinite;
int commandMux=0;
uint32_t clock_ms=100;
uint32_t millis(){return clock_ms;}
template<class T> T constrain(T v,T lo,T hi){return v<lo?lo:(v>hi?hi:v);}
struct Port {
  std::vector<uint8_t> queue;
  bool available(){return !queue.empty();}
  uint8_t read(){uint8_t v=queue.front();queue.erase(queue.begin());return v;}
} HostSerial;
MotionGuard motion;
uint32_t rxByteCount=0,rxValidCmdCount=0,rxBadChecksumCount=0,rxBadLenCount=0;
''' + macros + '\n' + _function(source, 'handleCommand') + '\n' +
                       _function(source, 'pollHost') + r'''
std::vector<uint8_t> frame(float l,float r) {
  std::vector<uint8_t> f={0xaa,0x55,1,8};
  uint8_t payload[8]; std::memcpy(payload,&l,4);std::memcpy(payload+4,&r,4);
  uint8_t sum=9;
  for(uint8_t v:payload){f.push_back(v);sum+=v;}
  f.push_back(sum);return f;
}
int main() {
  auto valid=frame(0.1f,0);
  HostSerial.queue={0x77,0xaa};pollHost();
  assert(rxValidCmdCount==0);
  HostSerial.queue.assign(valid.begin()+1,valid.end());pollHost();
  assert(rxValidCmdCount==1 && motion.valid);
  assert(std::fabs(motion.left-0.1f*60/(PI*WHEEL_DIAMETER_M))<1e-4);
  assert(motion.right==-motion.left);
  clock_ms=200;
  auto bad=frame(0,0);bad.back()^=1;HostSerial.queue=bad;pollHost();
  assert(rxBadChecksumCount==1 && motion.last_ms==100);
  HostSerial.queue=frame(std::numeric_limits<float>::quiet_NaN(),0);pollHost();
  HostSerial.queue=frame(0,std::numeric_limits<float>::infinity());pollHost();
  HostSerial.queue={0xaa,0x55,0x99,0,0x99};pollHost();
  HostSerial.queue={0xaa,0x55,1,65};pollHost();
  assert(rxValidCmdCount==1 && rxBadLenCount==4 && motion.last_ms==100);
  motion.expire(3099,CMD_TIMEOUT_MS);assert(motion.valid && motion.last_ms==100);
  motion.expire(3100,CMD_TIMEOUT_MS);assert(!motion.valid && motion.timed_out);
  // Abandon a truncated frame after a 50 ms gap, then accept a fresh zero frame.
  clock_ms=3200;HostSerial.queue={0xaa,0x55,1,8};pollHost();
  clock_ms=3260;HostSerial.queue=frame(0,0);pollHost();
  assert(rxValidCmdCount==2 && motion.valid && !motion.timed_out);
  HostSerial.queue={0xaa,0x55,2,0,2};pollHost();
  assert(rxValidCmdCount==3 && !motion.valid && motion.left==0 && motion.right==0);
  // Corrupt long frame enclosing valid movement, zero and stop frames. Preserve
  // every trailing valid frame when the parser resynchronizes inside that buffer.
  HostSerial.queue={0xaa,0x55,0x7f,64};
  for(auto command:{frame(0.1f,0),frame(0,0),std::vector<uint8_t>{0xaa,0x55,2,0,2}}) {
    HostSerial.queue.insert(HostSerial.queue.end(),command.begin(),command.end());
  }
  HostSerial.queue.resize(69,0xee);
  uint8_t sum=0;
  for(unsigned i=2;i<68;++i) sum+=HostSerial.queue[i];
  HostSerial.queue.back()=uint8_t(sum+1);
  pollHost();
  assert(rxValidCmdCount==6 && !motion.valid && motion.left==0 && motion.right==0);
}
''')
    executable = tmp_path / 'serial_commands'
    result = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra',
                             '-I', str(firmware), str(program), '-o', str(executable)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
