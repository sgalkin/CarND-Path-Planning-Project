#include "lane.h"
#include <limits>

// Open lane
LaneDescriptor::LaneDescriptor()
  : forward_limit(std::numeric_limits<float>::infinity())
  , forward_velocity(std::numeric_limits<float>::infinity())
  , backward_limit(-std::numeric_limits<float>::infinity())
  , backward_velocity(-std::numeric_limits<float>::infinity())
{}
    
LaneLimits evaluate(const Point& self, const std::vector<Vehicle>& v) {
  // TODO: this might have a problem when passing wraparound point
  LaneLimits ll;
  for(const auto& x: v) {
    auto lane = find_lane(x.frenet.y);
    auto& llimits = ll[lane];
    auto d = x.frenet.x - self.x;
    if(d >= 0 && d < llimits.forward_limit) {
      llimits.forward_limit = d;
      llimits.forward_velocity = magnitude(x.velocity);
    }
    if(d < 0 && d > llimits.backward_limit) {
      llimits.backward_limit = d;
      llimits.backward_velocity = magnitude(x.velocity);
    }
  }
  return ll;
}
