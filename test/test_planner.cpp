#include <catch.hpp>
#include <vector>
#include "planner.h"

TEST_CASE("Anchor") {
  std::vector<std::tuple<Point, float, Point>> m{
    std::make_tuple(Point{-5, 0}, -5.f, Point{0, 1}),
    std::make_tuple(Point{0, 0}, 0.f, Point{0, 1}),
    std::make_tuple(Point{5, 0}, 5.f, Point{0, 1}),
    std::make_tuple(Point{160, 0}, 160.f, Point{0, 1}),
  };
  Map map{begin(m), end(m)};
  
  Heading p{10, 6, 0};
  Point q = frenet::to(p, map);
  REQUIRE(p.x == Approx(q.x));
  REQUIRE(p.y == Approx(q.y));

//  Point r = frenet::from(q, map);
//  REQUIRE(p.x == Approx(r.x));
//  REQUIRE(p.y == Approx(r.y));
  
  SECTION("Keep lane") {
    Heading h{10, 6, 0};
    auto a = anchor(h, find_lane(h.y), map);
    auto d = a.front().y;
    for(const auto& wp: a) {
      REQUIRE(wp.y == Approx(d));
    }
  }
  SECTION("Change lane") {
    Heading h{10, 2, 0};
    auto a = anchor(h, 1, map);
    REQUIRE(a.back().y == Approx(-lane_center(1)));
    REQUIRE(a.front().y == Approx(-lane_center(0)));
    REQUIRE((a.begin() + a.size()/2)->y == Approx(-1/2.f*(lane_center(1)+lane_center(0))));
  }
  SECTION("Changing lane before lane border") {
    Heading h{10, 3.5f, 0};
    auto a = anchor(h, 1, map);
    REQUIRE(a.back().y == Approx(-lane_center(1)));
    REQUIRE(a.front().y == Approx(-3.5f));
  }
  SECTION("Changing lane before lane border") {
    Heading h{10, 3.5f, 0};
    auto a = anchor(h, 1, map);
    REQUIRE(a.back().y == Approx(-lane_center(1)));
    REQUIRE(a.front().y == Approx(-3.5f));
  }
  SECTION("Changing lane after lane border") {
    Heading h{10, 4.5f, 0};
    auto a = anchor(h, 1, map);
    REQUIRE(a.back().y == Approx(-lane_center(1)));
    REQUIRE(a.front().y == Approx(-4.5f));
  }
}
