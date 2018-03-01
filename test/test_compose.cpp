#include "catch.hpp"

#include <string>
#include "compose.h"

TEST_CASE("Compose") {
  auto sts = [](std::string a) { return a + a; };
  auto iti = [](int i) { return 4 - i; };
  auto sti = [](std::string s) { return static_cast<int>(s.size()); };

  auto f = compose(iti, sti, sts);
  REQUIRE(f("abc") == -2);
}
