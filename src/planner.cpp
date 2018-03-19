#include "planner.h"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <spline.h>

#include "point.h"
#include "util.h"
#include "limits.h"
#include "lane.h"
#include "coordinates.h"
#include "fusion.h"
#include "path.h"
#include "dump.h"

#include <iostream>


namespace {  
tk::spline spline(const Path& p) {
  tk::spline s;
  // TODO: suboptimal one loop will be better
  s.set_points(to_vector(p, [](const Point& p)->double{ return p.x; }),
               to_vector(p, [](const Point& p)->double{ return p.y; }));
  return s;
}
  
template<typename OutputIterator>
OutputIterator anchor(Point forigin, float d, const Map& m, OutputIterator o) {
  // able to change lane in desired time even if at 50% of speed_limit
  constexpr float offset = drive(1.f/2 * limits::speed_limit, limits::lane_change_time);
  constexpr size_t steps = 5;
  constexpr float horizon = steps * offset;

  size_t cnt = 0;
  return std::generate_n(o, steps,
                         [forigin, d, &m, &cnt]{
                           return frenet::from(
                             Point{
                               forigin.x + offset + (cnt++)*horizon/(steps - 1), d
                           }, m);
                         });
}

}

Path Planner::operator()(Model&& m) {
  DUMP(m, *map_);
  if(!lane_) {
    lane_.reset(new Lane(find_lane(m.ego.frenet.y)));
  }
  assert(lane_);
               
  Path path = prepend(m.path, m.ego.heading);
  assert(path.size() > 1);
  // origin == last point in known path
  const float origin_heading = heading(*next(path.rbegin()), *path.rbegin());
  
  const auto forigin = frenet::to(path.back(), origin_heading, *map_);
  const auto self_lane = find_lane(forigin.y);

//  {
  auto ll = lane_limits(forigin, estimate(m.fusion, limits::tick*m.path.size(), *map_));
  DUMP(ll);

  // Implicit FSM -> don't change lane during transition
  if(self_lane == lane_->id() &&
     std::fabs(forigin.y - lane_->center()) < 0.5f) {
    lane_->update(self_lane, ll);
  }
  
  Heading lorigin{path.back(), origin_heading};
  anchor(forigin, lane_->center(), *map_, std::back_inserter(path));

  std::transform(begin(path), end(path), begin(path),
                 [lorigin](const Path::value_type& p) {
                   return local::to(p, lorigin);
                 });
  auto s = spline(std::move(path));

  auto vref = velocity_(ll[lane_->id()].forward_velocity,
                        ll[lane_->id()].forward_limit);

  float x_upper_bound = (limits::path_size - m.path.size()) * drive((m.ego.velocity + vref) / 2, limits::tick);
  float d = magnitude(Point{x_upper_bound, float(s(x_upper_bound))});

  float x_step = x_upper_bound/(d/(limits::tick.count() * vref));
    
  Path r{m.path};
  float x = 0;
  std::generate_n(std::back_inserter(r), limits::path_size - r.size(),
                  [&s, &x, x_step, lorigin]() {
                    x += x_step;
                    return local::from(Point{x, float(s(x))}, lorigin);
                  });
  return r;
}
