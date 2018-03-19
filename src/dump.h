#include <iostream>

#include "model.h"
#include "map.h"
#include "lane.h"
#include "coordinates.h"

#if not defined(NDEBUG)
#define DUMP(...) do { dump(__VA_ARGS__); } while(false)
#else
#define DUMP(...) do { } while(false)
#endif

inline void dump(const Model& m, const Map& map) {
  std::cerr << "EGO" << m.ego.heading
            << ":v:" << m.ego.velocity
            << ";f:" << m.ego.frenet
            << ";d:" << m.destination
            << ";ps:" << m.path.size();
  if(m.path.size() > 1) {
    std::cerr << ";pb:" << frenet::to(*m.path.begin(), *next(m.path.begin()), map)
              << ";pe:" << frenet::to(*m.path.rbegin(), *next(m.path.rbegin()), map);
  }
  std::cerr << std::endl;
}

inline void dump(const LaneLimits& ll) {
  for(const auto& d: ll) {
    std::cerr << "fwl=" << d.forward_limit
              << ";fwv=" << d.forward_velocity
              << ";bwl=" << d.backward_limit
              << ";bwv=" << d.backward_velocity
              << "\n";
  }
}
