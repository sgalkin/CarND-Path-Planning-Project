#pragma once

#include <algorithm>
#include <vector>

#include "point.h"
#include "model.h"
#include "limits.h"
#include "util.h"

inline constexpr float drive(float v, Timestamp t) {
  return v*t.count();
}

inline constexpr Point drive(const Point& p, const Point& v, Timestamp t) {
  return p + Point{drive(v.x, t), drive(v.y, t)};
}

inline Point drive(const Heading& p, float v, Timestamp t) {
  return p + v*Point{drive(std::cos(p.theta), t), drive(std::sin(p.theta), t)};
}

