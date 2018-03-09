#pragma once

#include "map.h"
#include "util.h"
#include "frenet.h"
#include "limits.h"
#include "model.h"

class Planner {
public:
  explicit Planner(std::unique_ptr<Map> map)
    : map_(std::move(map))
  {}
  
  Path operator()(Model m) {
    Path p;

    float s = m.ego.frenet.x;
    float d = m.ego.frenet.y;
    for(size_t i = 0; i < 30; ++i) {
      Point next{s += 0.98f * limits::speed * limits::step.count(), d};
      p.emplace_back(frenet::from(std::move(next), *map_));
    }
    return p;
  }

private:
  std::unique_ptr<Map> map_;
};
