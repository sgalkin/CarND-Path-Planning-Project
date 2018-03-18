#pragma once

#include <memory>
#include <limits>

#include "map.h"
#include "model.h"
#include "path.h"

class Planner {
public:
  explicit Planner(std::unique_ptr<Map> map)
    : map_(std::move(map))
  {}
  
  Path operator()(Model&& m);

private:
  std::unique_ptr<Map> map_;
  float vref_{0};
  size_t lref_{std::numeric_limits<size_t>::max()};
};

