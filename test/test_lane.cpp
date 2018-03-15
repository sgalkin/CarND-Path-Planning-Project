#include "catch.hpp"
#include <limits>
#include "lane.h"

TEST_CASE("Lane") {
  SECTION("Find") {
    REQUIRE(0 == find_lane(0));
    REQUIRE(0 == find_lane(limits::lane_width/2));
    REQUIRE(0 == find_lane(limits::lane_width - 2*std::numeric_limits<float>::epsilon()));
    REQUIRE(1 == find_lane(limits::lane_width + 2*std::numeric_limits<float>::epsilon()));
  }
  SECTION("Center") {
    REQUIRE(Approx(5*limits::lane_width/2) == lane_center(2));
  }
};

TEST_CASE("LaneDescriptor") {
  std::vector<Vehicle> f{
    Vehicle{{0, 0}, {1, 0}, {-2, 2}},
    Vehicle{{0, 0}, {1, 1}, {-5, 7}},
    Vehicle{{-1, -1}, {0, 1}, {8, 5}},
    Vehicle{{20, 20}, {0, 0}, {3, 10}}
  };
  auto d = evaluate({3, 6}, f);
  REQUIRE(d[0].forward_limit == std::numeric_limits<float>::infinity());
  REQUIRE(d[0].forward_velocity == std::numeric_limits<float>::infinity());
  REQUIRE(d[0].backward_limit == Approx(5));
  REQUIRE(d[0].backward_velocity == Approx(1));

  REQUIRE(d[1].forward_limit == Approx(5));
  REQUIRE(d[1].forward_velocity == Approx(1));
  REQUIRE(d[1].backward_limit == Approx(8));
  REQUIRE(d[1].backward_velocity == Approx(std::sqrt(2.f)));

  REQUIRE(d[2].forward_limit == Approx(0));
  REQUIRE(d[2].forward_velocity == Approx(0));
  REQUIRE(d[2].backward_limit == std::numeric_limits<float>::infinity());
  REQUIRE(d[2].backward_velocity == -std::numeric_limits<float>::infinity());
}
