#pragma once

#include <fstream>
#include <chrono>
#include "ProgramOptions.hxx"

struct Config {
  const uint16_t port;
  const std::string waypoints;

  Config(int argc, char* argv[])
    : Config([](int argc, char* argv[]) {
        auto parser = Config::parser();
        if(!parser(argc, argv)) {
          std::cerr << parser << std::endl;
          throw std::runtime_error("invalid arguments");
        }
        return parser;
      }(argc, argv))
  {}
    
  explicit Config(po::parser p)
    : port{uint16_t(p["port"].get().u32)}
    , waypoints{p["waypoints"].get().string} {
      if(!std::ifstream(waypoints.c_str()).good()) {
        throw std::runtime_error("Unable to open file '" + waypoints + "'");
      }
    }

  static po::parser parser() {
    constexpr uint16_t port = 4567;
    constexpr const char* waypoints{"data/highway_map.csv"};

    po::parser p;
    p["help"].abbreviation('h')
      .description("print this help screen")
      .callback([&p]{ std::cerr << p << '\n'; exit(1); });
    p["port"].abbreviation('p').type(po::u32).fallback(port)
      .description("Port to use (default: " + std::to_string(port) + ")");
    p["waypoints"].abbreviation('w').type(po::string).fallback(waypoints)
      .description(std::string("File with route waypoints (defaults: ") + waypoints + ")");
    return p;
  }
};

template<typename OS>
OS& operator<< (OS& os, const Config& c) {
  os << "{"
     << "\"Port\":" << c.port << ","
     << "\"Waypoints\":\"" << c.waypoints << "\""
     << "}";
  return os;
}
