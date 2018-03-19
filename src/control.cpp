#include "control.h"

#include <iostream>
#include "limits.h"

float Velocity::operator()(float fw_velocity, float fw_gap) {
  if(fw_gap < limits::safe_minimal_gap) {
    std::cerr << "Safe gap reached. Slowing down!" << std::endl;
    // emergence break
    return velocity_ -= 0.2f;
  } else if(fw_gap < limits::safe_forward_gap &&
            velocity_ > fw_velocity) {
    // going to fast
    auto dv = std::max((velocity_ - fw_velocity)/64.f, 0.1f);
    // follow car with it's velocity
    return velocity_ = std::max(velocity_ - dv, fw_velocity);
  } else if(velocity_ < limits::max_speed) {
    // speed up!
    auto dv = std::max((limits::max_speed - velocity_)/32.f, 0.1f);
    return velocity_ = std::min(velocity_ + dv, limits::max_speed);
  }
  return velocity_;
}


