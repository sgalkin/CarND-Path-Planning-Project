#include "lane.h"

#include <cassert>
#include <iostream>
#include "limits.h"

namespace {
float cost(size_t self_lane, size_t target_lane, const LaneDescriptor& ld) {
  assert(self_lane < limits::lane_count &&
         target_lane < limits::lane_count &&
         std::max(self_lane, target_lane) - std::min(self_lane, target_lane) <= 1);
  
  bool keep_lane = self_lane == target_lane;
  if(ld.forward_limit < limits::safe_minimal_gap ||
     ld.backward_limit < limits::safe_minimal_gap) {
    // stay in lane if too close to car in front
    return keep_lane ? 0 : std::numeric_limits<float>::infinity();
  }
  
  float c = 0;
  // penalty for changings
  c += 0.25f * (!keep_lane);
  // penalty for using rightest lane
  c += 0.25f * (self_lane == 0); // compensate change lane penalty
  c += 0.02f * (target_lane == 0);
  c += 0.25f * limits::safe_forward_gap / ld.forward_limit;  // take in account free scpace in front
  c += 0.1f * (ld.forward_limit < 2*limits::safe_forward_gap) * limits::speed_limit / ld.forward_velocity; // take in account speed of the car in front
//  c += 0.05f * limits::safe_backward_gap / ld.backward_limit; // take in account free space behind 
  return c;
}
}

LaneLimits lane_limits(const Point& self, const std::vector<Vehicle>& v) {
  LaneLimits ll;
  for(const auto& x: v) {
    auto lane = find_lane(x.frenet.y);
    if(lane >= limits::lane_count) continue;
    
    auto& llimits = ll[lane];
    // TODO: this might have a problem when passing wraparound point
    auto d = x.frenet.x - self.x;
    if(d >= 0 && d < llimits.forward_limit) {
      llimits.forward_limit = d;
      llimits.forward_velocity = magnitude(x.velocity);
    }
    
    if(d < 0 && -d < llimits.backward_limit) {
      llimits.backward_limit = -d;
      llimits.backward_velocity = magnitude(x.velocity);
    }
  }
  return ll;
}

Lane::Lane(size_t lane) :
  lane_((assert(lane < limits::lane_count), lane))
{}

float Lane::center() const {
  return lane_center(id());
}

void Lane::update(size_t self_lane, const LaneLimits& limits) {
  float min_cost = std::numeric_limits<float>::infinity();
  size_t min_lane = 0;
  for(int i: { 0, -1, 1 }) {
    auto target_lane = self_lane + i;
    if(target_lane >= limits::lane_count) continue;
    assert(target_lane < limits::lane_count);
    
    auto lane_cost = cost(self_lane, target_lane, limits[target_lane]);
#if not defined(NDEBUG)
    std::cerr << "l=" << target_lane << ";c=" << lane_cost << "\n";
#endif
    if(lane_cost < min_cost) {
      min_cost = lane_cost;
      min_lane = target_lane;
    }
  }
  lane_ = min_lane;;
  assert(lane_ < limits::lane_count);
}
