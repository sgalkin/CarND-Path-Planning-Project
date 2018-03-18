#include <vector>
#include <memory>
#include <random>
#include "catch.hpp"

#include "map.h"
#include "coordinates.h"

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
    Heading h{p.x, p.y, float(a + M_PI/2)};
    
    auto f = frenet::to(std::move(h), *m);
    REQUIRE(f.x == Approx(R*a).margin(10));
    REQUIRE(f.y == Approx(r - R).margin(0.05));
    
    auto xy = frenet::from(f, *m);
    REQUIRE(p.x == Approx(xy.x).margin(10));
    REQUIRE(p.y == Approx(xy.y).margin(10));
  }
}

TEST_CASE("Current == 0") {
  constexpr float a = -0.0005/180*M_PI;
  constexpr float r = R - 2;

  auto p = c + r*Point{float(cos(a)), float(sin(a))}; 
  Heading h{p.x, p.y, a + float(M_PI/2)};
    
  auto f = frenet::to(std::move(h), *m);
  REQUIRE(f.x == Approx(R*a).margin(10));
  REQUIRE(f.y == Approx(r - R).margin(0.05));
    
  auto xy = frenet::from(f, *m);
  REQUIRE(p.x == Approx(xy.x).margin(10));
  REQUIRE(p.y == Approx(xy.y).margin(10));
}

TEST_CASE("Current == Last") {
  constexpr float a = -0.25/180*M_PI+2*M_PI;
  constexpr float r = R + 12;

  auto p = c + r*Point{float(cos(a)), float(sin(a))};
  Heading h{p.x, p.y, a + float(M_PI/2)};
    
  auto f = frenet::to(std::move(h), *m);
  REQUIRE(f.x == Approx(R*a).margin(10));
  REQUIRE(f.y == Approx(r - R).margin(0.05));
    
  auto xy = frenet::from(f, *m);
  REQUIRE(p.x == Approx(xy.x).margin(10));
  REQUIRE(p.y == Approx(xy.y).margin(10));
}

TEST_CASE("Forms") {
//  constexpr float a1 = 33/180*M_PI;
  constexpr float a2 = 34/180*M_PI;
  constexpr float r = R - 2;
//  auto p1 = c + r*Point{float(cos(a1)), float(sin(a1))};
  auto p2 = c + r*Point{float(cos(a2)), float(sin(a2))};
  Heading h2{p2, a2 + float(M_PI/2)};

  auto r1 = frenet::to(h2, *m);
  SECTION("point + angle") {
    auto r2 = frenet::to(p2, h2.theta, *m);
    REQUIRE(r2.x == Approx(r1.x));
    REQUIRE(r2.y == Approx(r2.y));
  }/*
  SECTION("point + point") {    
    auto r3 = frenet::to(p2, p1, *m);
    REQUIRE(r3.x == Approx(r1.x));
    REQUIRE(r3.y == Approx(r1.y));
    }*/
}

TEST_CASE("Cartesian") {
  Point p1{0, 0};
  Point p2{2, 0};
  Point p3{2, 2};
  Point p4{0, 2};

  Heading h{1, 1, M_PI/4};

  SECTION("Shift") {
    Heading h{1, 1, 0};
 
    auto c1 = local::to(p1, h);
    auto c2 = local::to(p2, h);
    auto c3 = local::to(p3, h);
    auto c4 = local::to(p4, h);
    auto cc = local::to(h, h);
    
    REQUIRE(c1.x == Approx(-1));
    REQUIRE(c1.y == Approx(-1));
    REQUIRE(c2.x == Approx(1));
    REQUIRE(c2.y == Approx(-1));
    REQUIRE(c3.x == Approx(1));
    REQUIRE(c3.y == Approx(1));
    REQUIRE(c4.x == Approx(-1));
    REQUIRE(c4.y == Approx(1));
    REQUIRE(cc.x == Approx(0));
    REQUIRE(cc.y == Approx(0));
  }
  
  SECTION("Forward") {
    auto c1 = local::to(p1, h);
    auto c2 = local::to(p2, h);
    auto c3 = local::to(p3, h);
    auto c4 = local::to(p4, h);
    auto cc = local::to(h, h);
    
    REQUIRE(c1.x == Approx(-std::sqrt(2)));
    REQUIRE(c1.y == Approx(0));
    REQUIRE(c2.x == Approx(0));
    REQUIRE(c2.y == Approx(-std::sqrt(2)));
    REQUIRE(c3.x == Approx(std::sqrt(2)));
    REQUIRE(c3.y == Approx(0));
    REQUIRE(c4.x == Approx(0));
    REQUIRE(c4.y == Approx(std::sqrt(2)));
    REQUIRE(cc.x == Approx(0));
    REQUIRE(cc.y == Approx(0));
  }
  
  SECTION("Reversible") {
    Point c1{local::from(local::to(p1, h), h)};
    Point c2{local::from(local::to(p2, h), h)};
    Point c3{local::from(local::to(p3, h), h)};
    Point c4{local::from(local::to(p4, h), h)};

    // epsilon doen't work on 0s
    REQUIRE(c1.x == Approx(p1.x).margin(1e-5));
    REQUIRE(c1.y == Approx(p1.y).margin(1e-5));
    REQUIRE(c2.x == Approx(p2.x).margin(1e-5));
    REQUIRE(c2.y == Approx(p2.y).margin(1e-5));
    REQUIRE(c3.x == Approx(p3.x).margin(1e-5));
    REQUIRE(c3.y == Approx(p3.y).margin(1e-5));
    REQUIRE(c4.x == Approx(p4.x).margin(1e-5));
    REQUIRE(c4.y == Approx(p4.y).margin(1e-5));
  }
}
