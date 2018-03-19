#pragma once

#include <memory>
#include <limits>

#include "map.h"
#include "model.h"
#include "path.h"
#include "control.h"
#include "lane.h"

class Planner {
public:
  explicit Planner(std::unique_ptr<Map> map)
    : map_(std::move(map))
  {}
  
  Path operator()(Model&& m);

private:
  std::unique_ptr<Map> map_;
  Velocity velocity_;
  std::unique_ptr<Lane> lane_;  // init first lane on update, might be std::optional in future
};

