#pragma once

#include <cmath>

struct alignas(16) Point {
  float x;
  float y;
};

template<typename IS>
IS& operator>> (IS& is, Point& p) {
  is >> p.x >> p.y;
  return is;
}

template<typename OS>
OS& operator<< (OS& os, const Point&p ) {
  return os << "x=" << p.x << ";y=" << p.y;
}


struct alignas(16) Heading : Point {
  Heading(float x, float y, float theta)
    : Point{x, y}
    , theta{theta}
  {}

  Heading(Point p, float theta)
    : Point{std::move(p)}
    , theta{theta}
  {}

  float theta;
};

template<typename IS>
IS& operator>> (IS& is, Heading& h) {
  return is >> static_cast<Point&>(h) >> h.theta;
}

template<typename OS>
OS& operator<< (OS& os, const Heading& h) {
  return os << static_cast<const Point&>(h) << ";theta=" << h.theta;
}


inline Point operator+ (Point lhs, Point rhs) {
  return Point{lhs.x + rhs.x, lhs.y + rhs.y};
}

inline Point operator* (float lhs, Point rhs) {
  return Point{lhs * rhs.x, lhs * rhs.y };
}

inline Point operator* (Point lhs, float rhs) {
  return rhs * std::move(lhs);
}

inline Point operator- (Point lhs) {
  return Point{-lhs.x, -lhs.y};
}

inline Point operator- (Point lhs, Point rhs) {
  return lhs + (-rhs);
}

inline Heading operator- (Heading lhs) {
  return Heading{-lhs.x, -lhs.y, -lhs.theta};
}

inline float distanceSquare(Point x, Point y) {
  auto d = x - y;
  return d.x*d.x + d.y*d.y;
}

inline float distance(Point x, Point y) {
  return std::sqrt(distanceSquare(x, y));
} 

inline float heading(Point src) {
  return std::atan2(src.y, src.x);
}

inline float heading(Point src, Point dst) {
  return heading(dst - src);
}


inline float magnitude(Point p) {
  return distance(p, Point{0, 0});
}



