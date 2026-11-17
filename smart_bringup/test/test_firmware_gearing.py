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
                                    'CMD_TIMEOUT_MS', 'LIDAR_PWM_CHANNEL', 'LIDAR_RUN_DUTY'))
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
int duty[5]={};
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
  assert(duty[LIDAR_PWM_CHANNEL]==0);  // No host command since startup.
  motion.accept(30,-30,100);
  clock_ms=110;
  pid_update();
  assert(motorState.pwm[0]>0 && motorState.pwm[1]<0);
  assert(duty[LIDAR_PWM_CHANNEL]==LIDAR_RUN_DUTY);
  // Do not call loop() or any serial routine: the PID task must stop on its own.
  clock_ms=3099;
  pid_update();
  assert(motion.valid && motorState.targets[0]==30 && motorState.targets[1]==-30);
  assert(duty[LIDAR_PWM_CHANNEL]==LIDAR_RUN_DUTY);
  clock_ms=3100;
  pid_update();
  assert(!motion.valid && motion.timed_out);
  assert(motorState.targets[0]==0 && motorState.targets[1]==0);
  assert(motorState.pwm[0]==0 && motorState.pwm[1]==0);
  for(int channel=0;channel<4;++channel) assert(duty[channel]==255);  // DRV8870 braking.
  assert(duty[LIDAR_PWM_CHANNEL]==0);
  // An idle host starts scanning without moving the wheels. Its 1 Hz keepalive
  // sustains scanning beyond the original lease, then disconnect stops it.
  for(clock_ms=4000;clock_ms<=8000;clock_ms+=1000) {
    motion.accept(0,0,clock_ms);
    pid_update();
    assert(duty[LIDAR_PWM_CHANNEL]==LIDAR_RUN_DUTY);
    assert(motorState.pwm[0]==0 && motorState.pwm[1]==0);
  }
  clock_ms=10999;pid_update();
  assert(duty[LIDAR_PWM_CHANNEL]==LIDAR_RUN_DUTY);
  clock_ms=11000;pid_update();
  assert(duty[LIDAR_PWM_CHANNEL]==0 && !motion.valid && !motion.timed_out);
  // Resumption and explicit STOP work independently of loop() and wheel speed.
  clock_ms=12000;motion.accept(0,0,clock_ms);pid_update();
  assert(duty[LIDAR_PWM_CHANNEL]==LIDAR_RUN_DUTY);
  motion.stop();clock_ms+=10;pid_update();
  assert(duty[LIDAR_PWM_CHANNEL]==0);
  // Millis rollover does not prematurely stop scanning or extend the timeout.
  clock_ms=0xfffffff0U;motion.accept(0,0,clock_ms);pid_update();
  assert(duty[LIDAR_PWM_CHANNEL]==LIDAR_RUN_DUTY);
  clock_ms=0xba7U;pid_update();
  assert(duty[LIDAR_PWM_CHANNEL]==LIDAR_RUN_DUTY);
  clock_ms=0xba8U;pid_update();
  assert(duty[LIDAR_PWM_CHANNEL]==0);
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
float commandLinear=0,commandAngular=0,ledLinear=0,ledAngular=0;
bool commandStopped=false,ledStandby=false;
uint32_t ledStamp=0;
void ws2812_update_motion(float linear,float angular,uint32_t now,bool standby) {
  ledLinear=linear;ledAngular=angular;ledStamp=now;ledStandby=standby;
}
void ws2812_show() {}
''' + macros + '\n' + _function(source, 'handleCommand') + '\n' +
                       _function(source, 'pollHost') + '\n' +
                       _function(source, 'pollLedMotion') + r'''
std::vector<uint8_t> frame(float l,float r) {
  std::vector<uint8_t> f={0xaa,0x55,1,8};
  uint8_t payload[8]; std::memcpy(payload,&l,4);std::memcpy(payload+4,&r,4);
  uint8_t sum=9;
  for(uint8_t v:payload){f.push_back(v);sum+=v;}
  f.push_back(sum);return f;
}
int main() {
  pollLedMotion(clock_ms);assert(ledLinear==0 && ledAngular==0 && ledStandby);
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
  pollLedMotion(clock_ms);assert(!ledStandby);  // Valid zero stays blue.
  HostSerial.queue={0xaa,0x55,2,0,2};pollHost();
  assert(rxValidCmdCount==3 && !motion.valid && motion.left==0 && motion.right==0);
  pollLedMotion(clock_ms);assert(!ledStandby);  // Explicit stop also stays blue.
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
  // Feed the exact validated, limited command to LEDs, then expire the copy
  // even if the independent PID task has not yet updated the shared guard.
  clock_ms=5000;HostSerial.queue=frame(9,-9);pollHost();pollLedMotion(clock_ms);
  assert(ledLinear==MAX_LINEAR_MPS && ledAngular==-MAX_ANGULAR_RADPS);
  assert(!ledStandby);
  clock_ms=7999;pollLedMotion(clock_ms);assert(ledLinear==MAX_LINEAR_MPS);
  clock_ms=8000;pollLedMotion(clock_ms);
  assert(ledLinear==0 && ledAngular==0 && ledStamp==8000 && motion.valid && ledStandby);
  // A bad frame cannot change the LED command or extend the lease.
  HostSerial.queue=frame(std::numeric_limits<float>::quiet_NaN(),0);pollHost();
  pollLedMotion(clock_ms);assert(ledLinear==0 && ledAngular==0);
  clock_ms=8100;HostSerial.queue=frame(-0.03f,0.2f);pollHost();pollLedMotion(clock_ms);
  assert(ledLinear==-0.03f && ledAngular==0.2f);
  HostSerial.queue={0xaa,0x55,2,0,2};pollHost();pollLedMotion(clock_ms);
  assert(ledLinear==0 && ledAngular==0 && !ledStandby);
  clock_ms=8200;HostSerial.queue=frame(0.03f,0);pollHost();
  clock_ms=9200;HostSerial.queue=frame(0.03f,0);pollHost();
  clock_ms=12199;pollLedMotion(clock_ms);assert(ledLinear==0.03f);
  clock_ms=12200;pollLedMotion(clock_ms);assert(ledLinear==0 && ledAngular==0);
  // A zero-speed lease expires too, despite the motion alarm staying clear.
  clock_ms=13000;HostSerial.queue=frame(0,0);pollHost();pollLedMotion(clock_ms);
  assert(!ledStandby);
  clock_ms=15999;pollLedMotion(clock_ms);assert(!ledStandby);
  clock_ms=16000;pollLedMotion(clock_ms);assert(ledStandby);
  HostSerial.queue={0xaa,0x55,2,0,2};pollHost();
  clock_ms=20000;pollLedMotion(clock_ms);assert(!ledStandby);
  clock_ms=0xfffffff0U;HostSerial.queue=frame(0,-0.2f);pollHost();
  clock_ms=0xba7U;pollLedMotion(clock_ms);assert(ledAngular==-0.2f);
  clock_ms=0xba8U;pollLedMotion(clock_ms);assert(ledLinear==0 && ledAngular==0);
}
''')
    executable = tmp_path / 'serial_commands'
    result = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra',
                             '-I', str(firmware), str(program), '-o', str(executable)],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
