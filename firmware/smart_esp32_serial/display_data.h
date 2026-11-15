#pragma once
#include <stdint.h>

struct FrontDistance {
  uint16_t range_mm = 0;
  uint32_t stamp_ms = 0;
  bool have_scan = false;

  // Corrected X2N raw angle 180 degrees is forward for this chassis mounting.
  template<typename Point>
  void update(const Point* points, uint16_t count, uint32_t now) {
    range_mm = 0;
    stamp_ms = now;
    have_scan = true;
    for (uint16_t i = 0; i < count; ++i) {
      const auto& point = points[i];
      if (point.angle_cdeg < 17500 || point.angle_cdeg > 18500 ||
          point.range_mm < 240 || point.range_mm > 10000) continue;
      if (!range_mm || point.range_mm < range_mm) range_mm = point.range_mm;
    }
  }

  bool valid(uint32_t now) const {
    return have_scan && range_mm && uint32_t(now-stamp_ms) < 1000;
  }
};
