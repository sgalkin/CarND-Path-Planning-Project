#pragma once

#include "limits.h"

class Velocity {
public:
  float operator()(float fw_velocity, float fw_gap);
private:
  float velocity_{0};
};
