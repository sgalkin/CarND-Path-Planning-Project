#include "catch.hpp"

#include <sstream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <unordered_set>
#include <iterator>
#include <Eigen/Core>
#include "json.hpp"
#include "io.h"

#if 0
namespace {
template<typename T>
std::string serialize(const std::vector<T>& v, const std::string s = "") {
  std::ostringstream os;
  std::copy(begin(v), end(v),
            std::ostream_iterator<T>(
              os << std::setprecision(std::numeric_limits<double>::digits10), s.c_str()));
  return os.str().substr(0, os.str().empty() ? std::string::npos : os.str().size() - 1);
}
}

TEST_CASE("Parse") {
  static const std::vector<double> pts_x{
    -32.16173,-43.49173,-61.09,-78.29172,-93.05002,-107.7717
  };
  static const std::vector<double> pts_y{
    113.361,105.941,92.88499,78.73102,65.34102,50.57938
  };
  
  static const std::string message{
    "[\"telemetry\", {"
      "\"psi\":3.733651,\"psi_unity\":4.12033,"
      "\"ptsx\":[" + serialize(pts_x, ",") + "],"
      "\"ptsy\":[" + serialize(pts_y, ",") + "],"
      "\"speed\":10,"
      "\"steering_angle\":-0.044,"
      "\"throttle\":0.3,"
      "\"x\":-40.62,"
      "\"y\":108.73"
    "}]"
  };
  
  Json j;
  auto m = j(message);
  REQUIRE(m.psi == Approx(3.733651));
//  REQUIRE(m.state.psiu == Approx(4.12033));
  REQUIRE(m.v == Approx(10));
  REQUIRE(m.p == Eigen::Vector2d(-40.62, 108.73));
  REQUIRE(m.current.angle == Approx(-0.044));
  REQUIRE(m.current.throttle == Approx(0.3));
  REQUIRE(m.wp.col(Axis::X) == Eigen::Map<const Eigen::VectorXd>(pts_x.data(), pts_x.size()));
  REQUIRE(m.wp.col(Axis::Y) == Eigen::Map<const Eigen::VectorXd>(pts_y.data(), pts_y.size()));
}

TEST_CASE("Compose") {
  Json j;
  SECTION("Keys") {
    static const std::unordered_set<std::string> known_keys{
      "steering_angle", "throttle", "mpc_x", "mpc_y", "next_x", "next_y"
    };
    Control c{
      1, 2,
      (Eigen::MatrixXd(4, 2) << 0, 1, 2, 3, 4, 5, 6, 7).finished(),
      (Eigen::MatrixXd(3, 2) << 0, -1, 2, -3, 4, -5).finished()
    };
    auto s = nlohmann::json::parse(j(c));
    for(auto it = begin(s); it != end(s); ++it) {
      REQUIRE(known_keys.count(it.key()));
    }
  }
  
  SECTION("FullValues") {
    Control c{
      1, 2,
      (Eigen::MatrixXd(4, 2) << 0, 1, 2, 3, 4, 5, 6, 7).finished(),
      (Eigen::MatrixXd(3, 2) << 0, -1, 2, -3, 4, -5).finished()
    };
    auto s = j(c);
    REQUIRE(s.find("\"steering_angle\":1.0") != std::string::npos);
    REQUIRE(s.find("\"throttle\":2.0") != std::string::npos);
    REQUIRE(s.find("\"mpc_x\":[0.0,2.0,4.0,6.0]") != std::string::npos);
    REQUIRE(s.find("\"mpc_y\":[1.0,3.0,5.0,7.0]") != std::string::npos);
    REQUIRE(s.find("\"next_x\":[0.0,2.0,4.0]") != std::string::npos);
    REQUIRE(s.find("\"next_y\":[-1.0,-3.0,-5.0]") != std::string::npos);    
  }
  
  SECTION("PartValues") {
    Control c{1, 2};
    auto s = j(c);
    REQUIRE(s.find("\"steering_angle\":1.0") != std::string::npos);
    REQUIRE(s.find("\"throttle\":2.0") != std::string::npos);    
  }  
}
#endif
