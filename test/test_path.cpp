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
