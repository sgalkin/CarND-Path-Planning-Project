#include "lane.h"

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
