#pragma once

#include <cmath>

struct alignas(16) Point {
  float x;
  float y;
};

template<typename IS>
IS& operator>> (IS& is, Point& p) {
  return is >> p.x >> p.y;
}

template<typename OS>
OS& operator<< (OS& os, const Point&p ) {
  return os << "x=" << p.x << ";y=" << p.y;
}


struct alignas(16) Heading : Point {
  float theta;
};

template<typename IS>
IS& operator>> (IS& is, Heading& h) {
  return is >> static_cast<Point&>(h) >> h.theta;
}

template<typename OS>
OS& operator<< (OS& os, const Heading& h) {
  return os << static_cast<const Heading&>(h) << ";theta=" << h.theta;
}


inline Point operator+ (Point lhs, Point rhs) {
  return Point{lhs.x + rhs.x, lhs.y + rhs.y};
}

inline Point operator* (float lhs, Point rhs) {
  return Point{lhs * rhs.x, lhs * rhs.y };
}

inline Point operator- (Point lhs, Point rhs) {
  return lhs + (-1)*rhs;
}


inline float distanceSquare(Point x, Point y) {
  auto d = x - y;
  return d.x*d.x + d.y*d.y;
}

inline float distance(Point x, Point y) {
  return sqrt(distanceSquare(x, y));
} 
