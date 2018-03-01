#include <iterator>
#include <fstream>

#include "catch.hpp"
#include "map.h"

TEST_CASE("Load") {
  std::ifstream map{"../data/highway_map.csv"};
  REQUIRE(map.good());
  Map m{
    std::istream_iterator<Map::Record>(map),
    std::istream_iterator<Map::Record>()
  };

  REQUIRE(m.size() == 181);

  // 784.6001 1135.571
  REQUIRE(m.begin()->x == Approx(784.6001));
  REQUIRE(m.begin()->y == Approx(1135.571));

  // 753.2067 1136.417
  REQUIRE(m.rbegin()->x == Approx(753.2067));
  REQUIRE(m.rbegin()->y == Approx(1136.417));

  REQUIRE(m.center().x == Approx(1428.5119));
  REQUIRE(m.center().y == Approx(2048.2344));
}

TEST_CASE("Nearest") {
  std::vector<Map::Record> data{
    std::make_tuple(Point{0, 0}, 0, Point{0, 0}),
    std::make_tuple(Point{0, 1}, 1, Point{0, 1})
  };
  Map m{begin(data), end(data)};
  REQUIRE(m.nearest(Point{-1, -1}) == 0);
  REQUIRE(m.nearest(Point{1, 1}) == 1);
}
