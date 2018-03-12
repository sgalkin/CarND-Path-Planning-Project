#pragma once

namespace limits {
  constexpr size_t lane_count = 3;
  constexpr float lane_width = 4;
}

constexpr size_t find_lane(float d) {
  return size_t((2*d - limits::lane_width)/(2*limits::lane_width) + 0.5f);
}

constexpr float lane_center(size_t idx) {
  return limits::lane_width/2 + idx*limits::lane_width;
}
