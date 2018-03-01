#include "catch.hpp"
#include <string>
#include "protocol.h"

TEST_CASE("getPayload") {
  SECTION("Wrong prefix") {
    std::string msg = "33[]";
    REQUIRE_THROWS(WSProtocol::getPayload(msg));
  }
  SECTION("Short prefix") {
    std::string msg = "4";
    REQUIRE_THROWS(WSProtocol::getPayload(msg));
  }

  SECTION("Null payload") {
    std::string msg = "42[\"aaa\": null]";
    REQUIRE(WSProtocol::getPayload(msg).empty());
  }
  SECTION("Broken payload [") {
    std::string msg = "42\"aaa\": 42]";
    REQUIRE(WSProtocol::getPayload(msg).empty());
  }
  SECTION("Broken payload ]") {
    std::string msg = "42[\"aaa\": 42";
    REQUIRE(WSProtocol::getPayload(msg).empty());
  }
  
  SECTION("Good payload") {
    std::string msg = "42[\"aaa\": 42] \"bb\": 44";
    REQUIRE(WSProtocol::getPayload(msg) == "[\"aaa\": 42]");
  } 
}

TEST_CASE("formatResponse") {
  SECTION("Manual") {
    REQUIRE(WSProtocol::formatResponse() == "42[\"manual\",{}]");
  }
  SECTION("Control") {
    REQUIRE(WSProtocol::formatResponse("{\"pay\": \"load\"}") == "42[\"control\",{\"pay\": \"load\"}]");
  }
}
