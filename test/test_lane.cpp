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
