#pragma once

#include <algorithm>
#include <vector>

#include "point.h"
#include "model.h"
#include "util.h"

inline Point drive(const Point& p, const Point& v, Timestamp t) {
  return Point{
    p.x + v.x*t.count(),
    p.y + v.y*t.count()
  };
}

inline Point drive(const Heading& p, float v, Timestamp t) {
  return Point{
    p.x + v*std::cos(p.theta)*t.count(),
    p.y + v*std::sin(p.theta)*t.count()
  };
}

inline Path prepend(Path p, const Heading& h) {
  if(p.size() >= 2) return p;
  Path{}.swap(p);
  p.insert(p.begin(), h);
  p.insert(p.begin(), drive(h, -limits::speed, limits::step));
  return p;
}

template<typename T>
Path append(Path p, const T& policy) {
  return policy(p);
}

template<typename F>
auto extract(const Path& p, F get)
  -> std::vector<typename std::result_of<F(const Path::value_type&)>::type> {
  std::vector<typename std::result_of<F(const Path::value_type&)>::type> r;
  r.reserve(p.size());
  std::transform(begin(p), end(p), std::back_inserter(r), get);
  return r;
}
