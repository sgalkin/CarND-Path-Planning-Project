#include "planner.h"

#include <algorithm>
#include <iterator>
#include <spline.h>
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

Path Planner::operator()(Model m) {
//  std::cout << "EGO:" << m.ego.heading
//            << " f:" << m.ego.frenet
//            << " ps:" << m.path.size() << std::endl;

  bool slow_down = false;
  std::vector<bool> lanes(true, 3);

  auto myl = find_lane(m.ego.frenet.y);
  transit = myl != lane;
  //if(!transit) clane = tlane;
  
  for(const auto& kv: m.fusion) {
//    if(find_lane(kv.second.frenet.y) != find_lane(m.ego.frenet.y)) {
//      continue;
//    }

    auto l = find_lane(kv.second.frenet.y);
    auto v = magnitude(kv.second.velocity);
    auto s = kv.second.frenet.x + v * limits::step.count() * m.path.size();

    if((s > m.current_destination.x && s - m.current_destination.x < 30) ||
       (s <= m.current_destination.x && m.current_destination.x - s < 20))
      lanes[l] = false;
    
    if(l == myl &&
       s > m.current_destination.x &&
       s - m.current_destination.x < /*limits::safe_gap*/30) {
      std::cerr << "Car ahead " << s - m.current_destination.x << " at speed: " << v << "\n";
      
      slow_down = true;

      if(v_ref > v) {
        auto dv = std::max((v_ref - v)/32.f, 0.1f);
        v_ref = std::max(v_ref - dv, v);
      }
      if(s - m.current_destination.x < 10) {
        std::cerr << "Slowing down!\n";
        // emergence break
        v_ref = v;
      }
    }
  }

  Path p = prepend(m.path, m.ego.heading);

  assert(p.size() > 1);
  Heading local_origin{*p.rbegin(), heading(*next(p.rbegin()), *p.rbegin())};

  Path a=p;
  bool change = false;
  float sss = frenet::to(*p.rbegin(), heading(*next(p.rbegin()), *p.begin()), *map_).x;
  sss += 10;
  if(!transit && slow_down) {
    for(int i = -1; i <= 1; ++i) {
      if((myl == 0 && i == -1) || (myl == 2 && i == +1)) continue;
      if(lanes[myl+i]) {
        std::cerr << "changing lanes!" << std::endl;
//        lane

        //a = append(std::move(p), ChangeLanePolicy<10, 30>(m.ego.heading, myl+i, *map_));
        a.push_back(frenet::from(Point{sss += 30, lane_center(lane)}, *map_));
        a.push_back(frenet::from(Point{sss += 30, (lane_center(lane) + lane_center(myl+i)) / 2.f}, *map_));
        a.push_back(frenet::from(Point{sss += 30, lane_center(myl+i)}, *map_));
        lane = myl+i;
        change = true;
        transit = true;
        //slow_down = false;
        break;
      }
    }
  }
  if(!change) {
//    a = append(std::move(p), KeepLanePolicy<10, 30, 3>(m.ego.heading, *map_));
    for(size_t i = 0; i < 3; ++i) {
      a.push_back(frenet::from(Point{sss += 30, lane_center(lane)}, *map_));
    }
  }
  std::transform(begin(a), end(a), begin(a),
                 [local_origin](const Path::value_type& p) {
                   return local::to(p, local_origin);
                 });
  auto s = spline(std::move(a));
  
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
