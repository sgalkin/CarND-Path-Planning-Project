#include "trajectory.h"

#include <spline.h>
#include "util.h"
#include "limits.h"
#include "coordinates.h"

namespace {
Path prepend(Path p, const Heading& h) {
  if(p.size() >= 2) return p;
  return Path{drive(h, -limits::speed_limit, limits::tick), h};
}

tk::spline spline(Path p, Heading lorigin) {
  std::transform(std::make_move_iterator(begin(p)),
                 std::make_move_iterator(end(p)),
                 begin(p),
                 [&lorigin](Path::value_type&& p) {
                   return local::to(std::move(p), lorigin);
                 });
  tk::spline s;
  // TODO: suboptimal one loop will be better
  s.set_points(to_vector(p, [](const Point& p)->double{ return p.x; }),
               to_vector(p, [](const Point& p)->double{ return p.y; }));
  return s;
}
  

Path anchor(Path base, Point forigin, float d, const Map& m) {
  // able to change lane in desired time even if at 50% of speed_limit
  constexpr float offset = drive(1.f/2 * limits::speed_limit, limits::lane_change_time);
  constexpr size_t steps = 5;
  constexpr float horizon = steps * offset;

  size_t cnt = 0;
  std::generate_n(std::back_inserter(base), steps,
                  [forigin, d, &m, &cnt]{
                    return frenet::from(
                      Point{ forigin.x + offset + (cnt++)*horizon/(steps - 1), d }, m
                    );
                  });
  return base;
}
}

Trajectory::Trajectory(Heading ego_heading, float ego_velocity, Path ego_path, const Map* map)
  : velocity_{ego_velocity}
  , base_path_{prepend(std::move(ego_path), ego_heading)}
  , local_{0, 0, 0} // TODO: not very good
  , map_{(assert(map), map)} {
  assert(base_path_.size() > 1);
  // origin == last point in known path
  const float origin_heading = heading(*next(base_path_.rbegin()), *base_path_.rbegin());
  frenet_ = frenet::to(base_path_.back(), origin_heading, *map_);
  local_ = Heading{base_path_.back(), origin_heading};
}

Path Trajectory::generate(Path p, size_t size, float center, float vref) const {
  auto reference = anchor(base_path_, frenet_, center, *map_);
  auto s = spline(std::move(reference), local_);

  float x_upper_bound = size*drive((velocity_ + vref) / 2, limits::tick);
  float distance = magnitude(Point{x_upper_bound, float(s(x_upper_bound))});

  float x_step = x_upper_bound/(distance/drive(vref, limits::tick));
  float x = 0;
  std::generate_n(std::back_inserter(p), size,
                  [&s, &x, x_step, this]() {
                    x += x_step;
                    return local::from(Point{x, float(s(x))}, local_);
                  });
  return p;
}
