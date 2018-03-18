#include "catch.hpp"

#include "path.h"

TEST_CASE("Drive") {
  SECTION("p, v") {
    Point p{0, 0};
    Point v{3, 4};
    Point r = drive(p, v, std::chrono::seconds{1});
    REQUIRE(r.x == v.x);
    REQUIRE(r.y == v.y);
  }
  SECTION("heading, magnitude") {
    Heading h{0, 0, M_PI/4};
    float v = std::sqrt(2.f);
    Point r = drive(h, v, std::chrono::seconds{1});
    REQUIRE(r.x == Approx(1));
    REQUIRE(r.y == Approx(1));
  }
}

TEST_CASE("Prepend") {
  SECTION("Long") {
    Path p{{0, 0}, {1, 1}, {2, 2}};
    Path q = prepend(p, {0, 0, 0});
    REQUIRE(q.size() == q.size());
    for (size_t i = 0; i < q.size(); ++i) {
      REQUIRE(q[i].x == p[i].x);
      REQUIRE(q[i].y == p[i].y);
    }
  }
  SECTION("Short") {
    Path p{{10, 10}};
    Path q = prepend(p, {0, 0, M_PI/6});
    REQUIRE(q.size() == 2);
    REQUIRE(q[0].x == Approx(-std::sqrt(3.f)/2*limits::speed_limit*limits::tick.count()));
    REQUIRE(q[0].y == Approx(-1.f/2*limits::speed_limit*limits::tick.count()));
    REQUIRE(q[1].x == 0);
    REQUIRE(q[1].y == 0);
  }
}
