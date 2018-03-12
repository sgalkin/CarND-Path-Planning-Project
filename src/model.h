#pragma once

#include <vector>
#include <unordered_map>
#include "point.h"
#include "path.h"

struct Ego {
  Ego(Heading h, double v, Point f)
    : heading(std::move(h))
    , velocity(std::move(v))
    , frenet(std::move(f))
  {}

  const Heading heading;
  const double velocity;
  const Point frenet;
};

struct Vehicle {
  Vehicle(Point p, Point v, Point f)
    : position(std::move(p))
    , velocity(std::move(v))
    , frenet(std::move(f))
  {}

  const Point position;
  const Point velocity;
  const Point frenet;
};

using Fusion = std::unordered_map<size_t, Vehicle>;


struct Model {
  Model(Ego e, Path p, Destination d, Fusion f)
    : ego(std::move(e))
    , path(std::move(p))
    , current_destination(std::move(d))
    , next_destination(current_destination)
    , fusion(std::move(f))
  {}
  
  const Ego ego;
  const Path path;
  const Destination current_destination;
  Destination next_destination;
  const Fusion fusion;
};
