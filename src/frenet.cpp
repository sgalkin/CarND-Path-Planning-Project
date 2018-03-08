#include "frenet.h"

#include <cmath>
#include <limits>
#include <tuple>
#include <vector>
#include <tuple>
#include <functional>

namespace {
inline std::size_t next(Heading h, const Map& m) {
  std::size_t closest = m.nearest(h);
  Point p = m.waypoint(closest) - h;
  float heading = atan2(p.y, p.x);
  
  float angle = fabs(h.theta - heading);
  angle = std::min(2*float(M_PI) - angle, angle);

  if(angle > M_PI/4)
    closest = closest == m.size() - 1 ? 0 : closest + 1;
  return closest;
}
}

namespace frenet {
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Point to(Heading h, const Map& m) {
  std::size_t next_wp = next(h, m);
  std::size_t prev_wp = next_wp == 0 ? m.size() - 1 : next_wp - 1;

  Point n = m.waypoint(next_wp) - m.waypoint(prev_wp);
  Point x = h - m.waypoint(prev_wp);

  // find the projection of x onto n
  float proj_norm = (x.x*n.x+x.y*n.y)/(n.x*n.x+n.y*n.y);
  Point p = proj_norm*n;

  float d = distance(x, p);
  //see if d value is positive or negative by comparing it to a center point
  Point center = m.center() - m.waypoint(prev_wp);
  if(distanceSquare(center, x) <= distanceSquare(center, p)) {
    d = std::copysign(d, -1);
  }

  // calculate s value
  float s = m.s(prev_wp) + distance(Point{0, 0}, p);
  return Point{s, d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
Point from(Point sd, const Map& m) {
  size_t wp1 = m.s_upper_bound(sd.x);
  size_t wp2 = wp1 == m.size() - 1 ? 0 : wp1 + 1;

  Point h = m.waypoint(wp2) - m.waypoint(wp1);
  float heading = atan2(h.y, h.x);

// the x,y,s along the segment
  float seg_s = (sd.x - m.s(wp1));
  Point seg = m.waypoint(wp1) + seg_s*Point{float(cos(heading)), float(sin(heading))};

  float perp = heading - M_PI/2;
  return seg + sd.y*Point{float(cos(perp)), float(sin(perp))};
}
}
