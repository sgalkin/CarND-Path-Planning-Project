#pragma once

#include <limits>
#include <array>

#include "model.h"

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

constexpr float lane_center(float d) {
  return lane_center(find_lane(d));
}


struct LaneDescriptor {    
  // TODO: use direction velocity not only magnitude
  float forward_limit{std::numeric_limits<float>::infinity()};
  float forward_velocity{std::numeric_limits<float>::infinity()};
  float backward_limit{std::numeric_limits<float>::infinity()};
  float backward_velocity{-std::numeric_limits<float>::infinity()};
};

using LaneLimits = std::array<LaneDescriptor, limits::lane_count>;

LaneLimits lane_limits(const Point& self, const std::vector<Vehicle>& v);

class Lane {
public:
  explicit Lane(size_t lane);

  size_t id() const { return lane_; }
  float center() const;

  void update(size_t self_lane, const LaneLimits& limits);
  
private:
  size_t lane_;
};
