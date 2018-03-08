#pragma once

#include <vector>
#include <unordered_map>
#include "point.h"

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

using Path = std::vector<Point>;
using Destination = Point;

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
    , destination(std::move(d))
    , fusion(std::move(f))
  {}

  const Ego ego;
  const Path path;
  const Destination destination;
  const Fusion fusion;
};
