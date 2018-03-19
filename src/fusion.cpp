#include "fusion.h"

#include <algorithm>

#include "path.h"
#include "coordinates.h"

Vehicle estimate(const Vehicle& v, Timestamp ts, const Map& map) {
  auto p = drive(v.position, v.velocity, ts);
  auto f = frenet::to(p, heading(v.velocity), map);
  return Vehicle{ std::move(p), v.velocity, std::move(f) };
}
  
std::vector<Vehicle> estimate(const Fusion& f, Timestamp ts, const Map& map) {
  std::vector<Vehicle> e;
  e.reserve(f.size());
  std::transform(begin(f), end(f), std::back_inserter(e),
                 [ts, &map](const Fusion::value_type& kv) {
                   return estimate(kv.second, ts, map);
                 });
  return e;
}
