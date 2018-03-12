#pragma once

#include <chrono>

inline constexpr float mph_to_ms(float mph) {
  constexpr float c = 0.44704;
  return mph*c;
}

inline constexpr float degrees_to_radians(float degrees) {
  return M_PI/180.f*degrees;
}

using Timestamp = std::chrono::duration<float>;

namespace limits {
  constexpr float acceleration = 10; // m/s^2
  constexpr float jerk = 10; // m/s^3
  constexpr auto lane_change{
    std::chrono::duration_cast<Timestamp>(std::chrono::seconds{3})
  };
  constexpr float speed = mph_to_ms(50); // m/s
  
  constexpr auto step{
    std::chrono::duration_cast<Timestamp>(std::chrono::milliseconds{20})
  }; // ms

  constexpr auto horizon{
    std::chrono::duration_cast<Timestamp>(std::chrono::seconds{1})
  }; // s

  constexpr size_t path_size = std::ceil(horizon/step);
}
