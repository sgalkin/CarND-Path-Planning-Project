#pragma once

#include <chrono>

namespace limits {
  constexpr float acceleration = 10; // m/s^2
  constexpr float jerk = 10; // m/s^3
  constexpr float lane_change = 3; // s
  constexpr float speed = 50; // mph

  constexpr auto step{
    std::chrono::duration_cast<std::chrono::duration<float>>(
      std::chrono::milliseconds{20})
    }; // ms
}


inline float mph_to_ms(float mph) {
  static constexpr float c = 0.44704;
  return mph*c;
}
