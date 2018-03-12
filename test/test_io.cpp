#include "catch.hpp"

#include <unordered_set>
#include <string>
#include "json.hpp"
#include "io.h"

TEST_CASE("JsonParse") {
  SECTION("Bad message. Header") {
    const std::string message = "[\"info:\", 42]";
    REQUIRE_THROWS(Json()(message));
  }
  SECTION("Bad message. Path mismatch") {
    const std::string message =
      "[\"telemetry\", {" 
      "\"x\": 909.48,"
      "\"y\": 1128.67,"
      "\"yaw\": -90,"
      "\"speed\": 30,"
      "\"s\": 124.8336,"
      "\"d\": 6.164833,"
      "\"previous_path_x\": [10, 11],"
      "\"previous_path_y\": [12],"
      "\"end_path_s\": 1044,"
      "\"end_path_d\": -2,"
      "\"sensor_fusion\": ["
      "[ 0,815.2019,1128.931,54.22955,-0.5962787,30.73384,5.999703],"
      "[ 3,775.8,1432.9,0,0,6713.911,-285.7268]"
      "]"
      "}]";
    REQUIRE_THROWS(Json()(message));
  }
  SECTION("Bad message. Invalid Fusion") {
    const std::string message =
      "[\"telemetry\", {" 
      "\"x\": 909.48,"
      "\"y\": 1128.67,"
      "\"yaw\": -90,"
      "\"speed\": 30,"
      "\"s\": 124.8336,"
      "\"d\": 6.164833,"
      "\"previous_path_x\": [10, 11],"
      "\"previous_path_y\": [12, 10],"
      "\"end_path_s\": 1044,"
      "\"end_path_d\": -2,"
      "\"sensor_fusion\": ["
      "[ 0,815.2019,1128.931,54.22955,-0.5962787,30.73384,5.999703,42],"
      "[ 3,775.8,1432.9,0,6713.911,-285.7268]"
      "]"
      "}]";
    REQUIRE_THROWS(Json()(message));
  }
  SECTION("Good message") {    
    const std::string message =
      "[\"telemetry\", {" 
      "\"x\": 909.48,"
      "\"y\": 1128.67,"
      "\"yaw\": -90,"
      "\"speed\": 30,"
      "\"s\": 124.8336,"
      "\"d\": 6.164833,"
      "\"previous_path_x\": [10, 11],"
      "\"previous_path_y\": [12, 21],"
      "\"end_path_s\": 1044,"
      "\"end_path_d\": -2,"
      "\"sensor_fusion\": ["
      "[ 0,815.2019,1128.931,54.22955,-0.5962787,30.73384,5.999703],"
      "[ 3,775.8,1432.9,0,0,6713.911,-285.7268]"
      "]"
      "}]";

    Json j;
    auto m = j(message);
    REQUIRE(m.ego.heading.x == Approx(909.48));
    REQUIRE(m.ego.heading.y == Approx(1128.67));
    REQUIRE(m.ego.heading.theta == Approx(-M_PI/2));
    REQUIRE(m.ego.velocity == Approx(13.4112));
    REQUIRE(m.ego.frenet.x == Approx(124.8336));
    REQUIRE(m.ego.frenet.y == Approx(6.164833));

    REQUIRE(m.path.size() == 2);
    REQUIRE(m.path[0].x == Approx(10));
    REQUIRE(m.path[0].y == Approx(12));
    REQUIRE(m.path[1].x == Approx(11));
    REQUIRE(m.path[1].y == Approx(21));

    REQUIRE(m.current_destination.x == Approx(1044));
    REQUIRE(m.current_destination.y == Approx(-2));

    REQUIRE(m.next_destination.x == Approx(1044));
    REQUIRE(m.next_destination.y == Approx(-2));

    REQUIRE(m.fusion.size() == 2);
    REQUIRE(m.fusion.at(0).position.x == Approx(815.2019));
    REQUIRE(m.fusion.at(0).position.y == Approx(1128.931));
    REQUIRE(m.fusion.at(0).velocity.x == Approx(54.22955));
    REQUIRE(m.fusion.at(0).velocity.y == Approx(-0.5962787));
    REQUIRE(m.fusion.at(0).frenet.x == Approx(30.73384));
    REQUIRE(m.fusion.at(0).frenet.y == Approx(5.999703));

    REQUIRE(m.fusion.at(3).position.x == Approx(775.8));
    REQUIRE(m.fusion.at(3).position.y == Approx(1432.9));
    REQUIRE(m.fusion.at(3).velocity.x == Approx(0));
    REQUIRE(m.fusion.at(3).velocity.y == Approx(0));
    REQUIRE(m.fusion.at(3).frenet.x == Approx(6713.911));
    REQUIRE(m.fusion.at(3).frenet.y == Approx(-285.7268));
  }
}

TEST_CASE("JsonCompose") {
  Json j;
  Path p{{0, 0}, {1, 2}};

  SECTION("Keys") {
    static const std::unordered_set<std::string> known_keys{
      "next_x", "next_y"
    };
    auto s = nlohmann::json::parse(j(p));
    for(auto it = begin(s); it != end(s); ++it) {
      REQUIRE(known_keys.count(it.key()));
    }
  }
  
  SECTION("Values") {
    auto s = j(p);
    REQUIRE(s.find("\"next_x\":[0.0,1.0]") != std::string::npos);
    REQUIRE(s.find("\"next_y\":[0.0,2.0]") != std::string::npos);    
  }

  SECTION("NoValues") {
    auto e = j(Path{});
    REQUIRE(e.find("\"next_x\":[]") != std::string::npos);
    REQUIRE(e.find("\"next_y\":[]") != std::string::npos);    
  }
}
