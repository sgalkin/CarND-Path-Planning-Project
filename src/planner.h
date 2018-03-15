#pragma once

#include <vector>
#include <memory>

#include "point.h"
#include "map.h"
#include "model.h"
#include "path.h"
#include "coordinates.h"
#include "lane.h"
#include <iostream>

template<size_t Offset, size_t Step, size_t Count>
class KeepLanePolicy {
public:
  explicit KeepLanePolicy(const Heading& h, size_t target, const Map& m)
  //    : lane_{find_lane(frenet::to(h, m).y)}
    : lane_(target)
    , map_(m)
  {}

  Path operator()(Path p) const {
    assert(p.size() > 1);
    float s = frenet::to(*p.rbegin(), heading(*next(p.rbegin()), *p.begin()), map_).x;
    s += Offset;
    for(size_t i = 0; i < Count; ++i) {
      p.push_back(frenet::from(Point{s += Step, lane_center(lane_)}, map_));
    }
    return p;
  }
  
private:
  size_t lane_;
  const Map& map_;
};

template<size_t Offset, size_t Step>
class ChangeLanePolicy {
public:
  explicit ChangeLanePolicy(const Heading& h, size_t target, const Map& m)
    : lane_{find_lane(frenet::to(h, m).y)}
    , target_(target)
    , map_(m)
  {}

  Path operator()(Path p) const {
    assert(p.size() > 1);
    float s = frenet::to(*p.rbegin(), heading(*next(p.rbegin()), *p.begin()), map_).x;
    s += Offset;
    p.push_back(frenet::from(Point{s += Step, lane_center(lane_)}, map_));
    p.push_back(frenet::from(Point{s += Step, (lane_center(lane_) + lane_center(target_)) / 2.f}, map_));
    p.push_back(frenet::from(Point{s += Step, lane_center(target_)}, map_));
    return p;
  }
  
private:
  size_t lane_;
  size_t target_;
  const Map& map_;  
};

inline Path anchor(Heading src, uint8_t target, const Map& m) {
  constexpr float offset = 3;
  constexpr float horizon = 90;
  constexpr size_t steps = 5;

  Point f = frenet::to(src, m);

  Path p;
  for(size_t i = 0; i < steps; ++i) {
    p.emplace_back(frenet::from(Point{
      f.x + offset + i*horizon/(steps - 1),
      ((steps - 1 - i)*f.y + i*lane_center(target))/(steps - 1)
    }, m));
  }
  return p;
}

class Planner {
public:
  explicit Planner(std::unique_ptr<Map> map)
    : map_(std::move(map))
  {}
  
  Path operator()(Model m);

private:
//  size_t lane_{size_t(-1)};
  std::unique_ptr<Map> map_;
};
