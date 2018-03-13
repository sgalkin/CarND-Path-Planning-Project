#include "planner.h"

#include <algorithm>
#include <iterator>
#include <spline.h>
#include "point.h"
#include "util.h"
#include "limits.h"

#include <iostream>

namespace {
  
tk::spline spline(const Path& p) {
  tk::spline s;
  // TODO: suboptimal one loop will be better
  s.set_points(extract(p, [](const Point& p)->double{ return p.x; }),
               extract(p, [](const Point& p)->double{ return p.y; }));
  return s;
}

//float v_max = 0.99f * limits::speed;

float v_ref = 0;
//  size_t clane = (size_t)-1;
size_t lane = (size_t)1;
  
bool transit = false;  
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


Path Planner::operator()(Model m) {
  auto ml = find_lane(/*m.ego.frenet.y*/m.destination.y);
  std::cout << "EGO:" << m.ego.heading
            << " f:" << m.ego.frenet
            << " ps:" << m.path.size()
            << " ml:" << find_lane(m.ego.frenet.y)
            << " mlf:" << ml
            << std::endl;
  
  auto e = estimate(m.fusion, limits::step*m.path.size(), *map_);
  auto ll = evaluate(m.destination, std::move(e));
  size_t i = 0;
  for(const auto& d: ll) {
    std::cerr << (ml == i++ ? "* " : "  ")
              << "fwl=" << d.forward_limit
              << ";fwv=" << d.forward_velocity
              << ";bwl=" << d.backward_limit
              << ";bwv=" << d.backward_velocity
              << "\n";
  }

  bool slow_down = false;
  std::vector<bool> lanes(true, 3);

  transit = ml != lane;
  //if(!transit) clane = tlane;
  
  for(const auto& kv: m.fusion) {
//    if(find_lane(kv.second.frenet.y) != find_lane(m.ego.frenet.y)) {
//      continue;
//    }

    auto l = find_lane(kv.second.frenet.y);
    auto v = magnitude(kv.second.velocity);
    auto s = kv.second.frenet.x + v * limits::step.count() * m.path.size();

    if((s > m.destination.x && s - m.destination.x < 30) ||
       (s <= m.destination.x && m.destination.x - s < 20))
      lanes[l] = false;
    
    if(l == ml &&
       s > m.destination.x &&
       s - m.destination.x < /*limits::safe_gap*/30) {
      std::cerr << "Car ahead " << s - m.destination.x << " at speed: " << v << "\n";
      
      slow_down = true;

      if(v_ref > v) {
        auto dv = std::max((v_ref - v)/32.f, 0.1f);
        v_ref = std::max(v_ref - dv, v);
      }
      if(s - m.destination.x < 10) {
        std::cerr << "Slowing down!\n";
        // emergence break
        v_ref = v;
      }
    }
  }

  Path p = prepend(m.path, m.ego.heading);

  assert(p.size() > 1);
  Heading local_origin{*p.rbegin(), heading(*next(p.rbegin()), *p.rbegin())};

  //Path a=p;
  bool change = false;
  //float sss = frenet::to(*p.rbegin(), heading(*next(p.rbegin()), *p.begin()), *map_).x;
  //sss += 10;
  if(!transit && slow_down) {
    for(int i = -1; i <= 1; ++i) {
      if((ml == 0 && i == -1) || (ml == 2 && i == +1)) continue;
      if(lanes[ml+i]) {
        std::cerr << "changing lanes!" << std::endl;
//        lane
/*
        a.push_back(frenet::from(Point{sss += 30, lane_center(lane)}, *map_));
        a.push_back(frenet::from(Point{sss += 30, (lane_center(lane) + lane_center(ml+i)) / 2.f}, *map_));
        a.push_back(frenet::from(Point{sss += 30, lane_center(ml+i)}, *map_));
        lane = ml+i;
*/
        Path a = anchor(local_origin, ml, ml+i, *map_);
        std::move(begin(a), end(a), std::back_inserter(p));
        change = true;
        transit = true;
        lane = ml+i;
        //slow_down = false;
        break;
      }
    }
  }
  if(!change) {
    //for(size_t i = 0; i < 3; ++i) {
      //a.push_back(frenet::from(Point{sss += 30, lane_center(lane)}, *map_));
    //}
    Path a = anchor(local_origin, ml, ml, *map_);
    std::move(begin(a), end(a), std::back_inserter(p));
  }
  std::transform(begin(p), end(p), begin(p),
                 [local_origin](const Path::value_type& p) {
                   return local::to(p, local_origin);
                 });
  auto s = spline(std::move(p));
  
  // speed control
  if(!slow_down && v_ref < 0.98f*limits::speed) {
    auto dv = std::max((limits::speed - v_ref)/32.f, 0.1f);
    v_ref = std::min(v_ref + dv, 0.98f*limits::speed);
  }

  constexpr float x_upper_bound = limits::speed * limits::horizon.count();
  float y_upper_bound = s(x_upper_bound);
  float d = magnitude(Point{x_upper_bound, y_upper_bound});

  float x_step = x_upper_bound/(d/(limits::step.count() * v_ref));
    
  Path r{m.path};
  float x = 0;
  std::generate_n(std::back_inserter(r), limits::path_size - r.size(),
                  [&s, &x, x_step, local_origin]() {
                    x += x_step;
                    return local::from(Point{x, float(s(x))}, local_origin);
                  });
  return r;
}
