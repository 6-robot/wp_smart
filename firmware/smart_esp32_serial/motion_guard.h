#pragma once
#include <stdint.h>

// Access under commandMux. Only validated CMD_VEL packets renew the lease.
struct MotionGuard {
  float left = 0, right = 0;
  uint32_t last_ms = 0;
  bool valid = false, timed_out = false;
  void accept(float l, float r, uint32_t now) {
    left = l; right = r; last_ms = now; valid = true; timed_out = false;
  }
  void stop() { left = right = 0; valid = false; }
  void expire(uint32_t now, uint32_t timeout) {
    if (valid && uint32_t(now-last_ms) >= timeout) {
      bool moving = left != 0 || right != 0;
      stop();
      if (moving) timed_out = true;
    }
  }
};
