#include "catch.hpp"

#include <sstream>
#include "point.h"

TEST_CASE("Point") {
  SECTION("Input") {
    std::istringstream s("10 42");
    Point p;
    s >> p;
    REQUIRE(p.x == Approx(10));
    REQUIRE(p.y == Approx(42));
  }
  SECTION("+") {
    Point p{1, 2};
    Point q{3, 4};
    Point r = p + q;
    REQUIRE(r.x == p.x + q.x);
    REQUIRE(r.y == p.y + q.y);
  }
  SECTION("-") {
    Point p{1, 2};
    Point q{3, 4};
    Point r = p - q;
    REQUIRE(r.x == p.x - q.x);
    REQUIRE(r.y == p.y - q.y);
  }
  SECTION("*") {
    Point p{1, 2};
    float q = 42.;
    Point r = p * q;
    REQUIRE(r.x == p.x * q);
    REQUIRE(r.y == p.y * q);
  }
  SECTION("negation") {
    Point p{1, 2};
    REQUIRE((-p).x == -p.x);
    REQUIRE((-p).y == -p.y);
  }
  SECTION("distance") {
    Point p{0, 0};
    Point q{3, 4};
    REQUIRE(distance(p, q) == Approx(5));
  }
  SECTION("heading") {
    Point p{-1, -1};
    Point q{-1 + std::sqrt(3.f)/2, -1 + 1.f/2};
    REQUIRE(heading(p, q) == Approx(M_PI/6));
  }
  SECTION("magnitude") {
    Point p{3, 4};
    REQUIRE(magnitude(p) == Approx(5));
  }
}

TEST_CASE("Heading") {
  SECTION("From x,y,theta") {
    Heading h{1, 2, M_PI/3};
    REQUIRE(h.x == Approx(1));
    REQUIRE(h.y == Approx(2));
    REQUIRE(h.theta == Approx(M_PI/3));
  }
  SECTION("From point,theta") {
    Heading h{Point{5, 3}, M_PI/4};
    REQUIRE(h.x == Approx(5));
    REQUIRE(h.y == Approx(3));
    REQUIRE(h.theta == Approx(M_PI/4));
  }
}
