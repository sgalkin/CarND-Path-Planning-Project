#include "planner.h"

#include <algorithm>
#include <iterator>
#include <spline.h>

#include "point.h"
#include "util.h"
#include "limits.h"
#include "lane.h"
#include "coordinates.h"

#include <iostream>

namespace limits {
  constexpr float safe_minimal_gap = 20;
  constexpr float safe_forward_gap = 30;
  constexpr float safe_backward_gap = 10;
}

namespace {  
tk::spline spline(const Path& p) {
  tk::spline s;
  // TODO: suboptimal one loop will be better
  s.set_points(extract(p, [](const Point& p)->double{ return p.x; }),
               extract(p, [](const Point& p)->double{ return p.y; }));
  return s;
}

std::vector<Vehicle> estimate(const Fusion& f, Timestamp ts, const Map& map) {
  std::vector<Vehicle> e;
  std::transform(begin(f), end(f), std::back_inserter(e),
                 [ts, &map](const Fusion::value_type& kv) {
                   auto p = drive(kv.second.position, kv.second.velocity, ts);
                   auto f = frenet::to(p, heading(kv.second.velocity), map);
                   return Vehicle{ std::move(p), kv.second.velocity, std::move(f) };
                 });
  return e;
}

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
  c += 0.25f * (!keep_lane);         // penalty for changings
  c += 0.02f * (self_lane == 0);     // penalty for using rightest lane
  c += 0.25f * limits::safe_forward_gap / ld.forward_limit;  // take in account free scpace in front
  c += 0.1f * (ld.forward_limit < 2*limits::safe_forward_gap) * limits::speed / ld.forward_velocity; // take in account speed of the car in front
  c += 0.05f * limits::safe_backward_gap / ld.backward_limit; // take in account free space behind 
  return c;
}

template<typename OutputIterator>
OutputIterator anchor(Point forigin, float d, const Map& m, OutputIterator o) {
  constexpr float offset = 0.5 * limits::lane_change.count() * limits::speed;
  constexpr size_t steps = 4;
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
  if(lref_ >= limits::lane_count) lref_ = find_lane(m.ego.frenet.y);

  Path path = prepend(m.path, m.ego.heading);
  assert(path.size() > 1);
  // origin == last point in known path
  const float origin_heading = heading(*next(path.rbegin()), *path.rbegin());
  
  const auto forigin = frenet::to(path.back(), origin_heading, *map_);
  std::cerr << "fo=" << forigin << "\n";
  const auto self_lane = find_lane(forigin.y);
  std::cerr << "sl=" << self_lane << "\n";

#if not defined(NDEBUG)
  do {
    std::cerr << "EGO" << m.ego.heading
              << ":v:" << m.ego.velocity
              << ";f:" << m.ego.frenet
              << ";d:" << m.destination
              << ";ps:" << m.path.size();
    if(m.path.size() > 1) {
      std::cerr << ";pb:" << frenet::to(*m.path.begin(), *next(m.path.begin()), *map_)
                << ";pe:" << frenet::to(*m.path.rbegin(), *next(m.path.rbegin()), *map_);
    }
    std::cerr << std::endl;
  } while(false);
#endif
//  {
  auto e = estimate(m.fusion, limits::step*m.path.size(), *map_);
  auto ll = lane_limits(forigin, std::move(e));
#if not defined(NDEBUG)
  do {
    size_t i = 0;
    for(const auto& d: ll) {
      std::cerr << (self_lane == i++ ? " * " : "   ")
                << "fwl=" << d.forward_limit
                << ";fwv=" << d.forward_velocity
                << ";bwl=" << d.backward_limit
                << ";bwv=" << d.backward_velocity
                << "\n";
    }
  } while(false);
#endif
  if(self_lane == lref_ &&
     std::fabs(forigin.y - lane_center(lref_)) < 0.5f) {

    float c = std::numeric_limits<float>::infinity();
    size_t l = 0;
    for(int i: { 0, -1, 1 }) {
      auto target_lane = self_lane + i;
      if(target_lane >= limits::lane_count) {
        continue;
      }
      assert(target_lane < limits::lane_count);
      auto lane_cost = cost(self_lane, target_lane, ll[target_lane]);
#if not defined(NDEBUG)
      std::cerr << "l=" << target_lane << ";c=" << lane_cost << "\n";
#endif
      if(lane_cost < c) {
        c = lane_cost;
        l = target_lane;
      }
    }
    lref_ = l;
  }
#if not defined(NDEBUG)
  std::cerr << "lr=" << lref_ << "\n";
#endif
  assert(lref_ < limits::lane_count);
//  }

  Heading lorigin{path.back(), origin_heading};
  anchor(forigin, lane_center(lref_), *map_, std::back_inserter(path));
  std::cerr << "lo=" << lorigin << "\n";
  std::transform(begin(path), end(path), begin(path),
                 [lorigin](const Path::value_type& p) {
                   std::cerr << "gp=" << p << "\n";
                   return local::to(p, lorigin);
                 });
  auto s = spline(std::move(path));

  {  
  auto fv = ll[lref_].forward_velocity;
  auto fg = ll[lref_].forward_limit;
  if(fg < limits::safe_minimal_gap) {
    std::cerr << "Slowing down!\n";
    // emergence break
    vref_ -= 0.2f;
  } else if(//fg < limits::safe_minimal_gap ||
            (fg < limits::safe_forward_gap && vref_ > fv)) {
    auto dv = std::max((vref_ - fv)/64.f, 0.1f);
    std::cerr << "dv1=" << dv << "\n";
    vref_ = std::max(vref_ - dv, fv); // follow car with it's velocity
    //vref_ -= dv;
  } else if(vref_ < limits::max_speed) {
    auto dv = std::max((limits::max_speed - vref_)/32.f, 0.1f);
    std::cerr << "dv2=" << dv << "\n";
    vref_ = std::min(vref_ + dv, limits::max_speed);
  }
  std::cerr << "vr=" << vref_ << "\n";
  }

  /*constexpr */
//  float x_upper_bound = std::max(30.f, 0.75f * limits::lane_change.count() * vref_);//limits::speed * limits::horizon.count();
  constexpr float x_upper_bound = 40; //limits::speed * limits::step.count();
  float y_upper_bound = s(x_upper_bound);
  float d = magnitude(Point{x_upper_bound, y_upper_bound});

  float x_step = x_upper_bound/(d/(limits::step.count() * vref_));
    
  Path r{m.path};
  float x = 0;
  std::generate_n(std::back_inserter(r), limits::path_size - r.size(),
                  [&s, &x, x_step, lorigin]() {
                    x += x_step;
                    return local::from(Point{x, float(s(x))}, lorigin);
                  });
  return r;
}
