#include <vector>
#include <memory>
#include <random>
#include "catch.hpp"

#include "map.h"
#include "frenet.h"

namespace {
  constexpr float R = 1000;
  Point c{1000, 2000};
  
  std::unique_ptr<Map> generate(float step_degrees=0.25) {
    std::vector<Map::Record> r;
    for(float a = 0; a < 2*M_PI; a += M_PI/180*step_degrees) {
      auto ca = float(cos(a));
      auto sa = float(sin(a));
      r.emplace_back(c + R*Point{ca, sa}, R*a, Point{ca, sa});
    }
    return std::unique_ptr<Map>(new Map(begin(r), end(r)));
  }

  auto m = generate();
  std::mt19937 gen(42);
  std::uniform_real_distribution<> da(0, 2*M_PI);
  std::normal_distribution<> dr(R, 5);
}

TEST_CASE("Circle") {
  for(size_t i = 0; i < 10000; ++i) {
    auto a = da(gen);
    auto r = dr(gen);

    auto p = c + r*Point{float(cos(a)), float(sin(a))};
    Heading h;
    h.x = p.x;
    h.y = p.y;
    h.theta = a + M_PI/2;
    
    auto f = to::frenet(std::move(h), *m);
    REQUIRE(f.x == Approx(R*a).margin(10));
    REQUIRE(f.y == Approx(r - R).margin(0.05));
    
    auto xy = to::xy(f, *m);
    REQUIRE(p.x == Approx(xy.x).margin(10));
    REQUIRE(p.y == Approx(xy.y).margin(10));
  }
}

TEST_CASE("Current == 0") {
  float a = -0.0005/180*M_PI;
  float r = R - 2;

  auto p = c + r*Point{float(cos(a)), float(sin(a))};
  Heading h;
  h.x = p.x;
  h.y = p.y;
  h.theta = a + M_PI/2;
    
  auto f = to::frenet(std::move(h), *m);
  REQUIRE(f.x == Approx(R*a).margin(10));
  REQUIRE(f.y == Approx(r - R).margin(0.05));
    
  auto xy = to::xy(f, *m);
  REQUIRE(p.x == Approx(xy.x).margin(10));
  REQUIRE(p.y == Approx(xy.y).margin(10));
}

TEST_CASE("Current == Last") {
  float a = -0.25/180*M_PI+2*M_PI;
  float r = R + 12;

  auto p = c + r*Point{float(cos(a)), float(sin(a))};
  Heading h;
  h.x = p.x;
  h.y = p.y;
  h.theta = a + M_PI/2;
    
  auto f = to::frenet(std::move(h), *m);
  REQUIRE(f.x == Approx(R*a).margin(10));
  REQUIRE(f.y == Approx(r - R).margin(0.05));
    
  auto xy = to::xy(f, *m);
  REQUIRE(p.x == Approx(xy.x).margin(10));
  REQUIRE(p.y == Approx(xy.y).margin(10));
}
