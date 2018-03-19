#include "planner.h"

#include <cassert>

#include "limits.h"
#include "lane.h"
#include "fusion.h"
#include "path.h"
#include "dump.h"
#include "trajectory.h"

Path Planner::operator()(Model&& m) {
  DUMP(m, *map_);
  
  if(!lane_) {
    if(find_lane(m.ego.frenet.y) < limits::lane_count) {
      lane_.reset(new Lane(find_lane(m.ego.frenet.y)));
    } else {
      return m.path;
    }
  }
  assert(lane_);

  Trajectory t{m.ego.heading, m.ego.velocity, m.path, map_.get()};
  
  const auto forigin = t.frenet_origin();
  const auto self_lane = find_lane(forigin.y);

  auto ll = lane_limits(forigin,
                        estimate(std::move(m.fusion),
                                 limits::tick*m.path.size(), *map_));
  DUMP(ll);

  // Implicit FSM -> don't change lane during transition
  if(self_lane == lane_->id() &&
     std::fabs(forigin.y - lane_->center()) < 1.f) {
    lane_->update(self_lane, ll);
  }
  auto vref = velocity_(ll[lane_->id()].forward_velocity,
                        ll[lane_->id()].forward_limit);

  return t.generate(std::move(m.path),
                    limits::path_size - m.path.size(),
                    lane_->center(), vref);
}
