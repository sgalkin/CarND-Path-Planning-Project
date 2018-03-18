#include "io.h"

#include <algorithm>
#include "path.h"
#include "json.hpp"
#include "util.h"

using json = nlohmann::json;

namespace {
constexpr float degrees_to_radians(float degrees) {
  return M_PI/180.f*degrees;
}
  
json extract(std::string message) {
  auto j = json::parse(std::move(message));
  if (j[0].get<std::string>() != "telemetry") {
    throw std::runtime_error("Unexpected event type");
  }
  return j[1];
}

template<typename U, typename V>
std::vector<U> to_vector(const std::vector<V>& p, std::function<U(const V&)> x) {
  std::vector<U> r(p.size());
  std::transform(begin(p), end(p), begin(r), x);
  return r;
}

std::vector<Point> to_vector(const std::vector<float>&& x,
                             const std::vector<float>&& y) {
  assert(x.size() == y.size());
  std::vector<Point> r(x.size());
  std::transform(begin(x), end(x), begin(y), begin(r),
                 [](float x, float y) { return Point{x, y}; });
  return r;
}

Ego ego(const json& j) {
  return Ego{
    Heading{
      j["x"].get<float>(), j["y"].get<float>(),
      degrees_to_radians(j["yaw"].get<float>())
    },
    mph_to_ms(j["speed"].get<float>()),
    Point{
      j["s"].get<float>(), j["d"].get<float>()
    }
  };
}

Path path(const json& j) {
  const auto& x = j["previous_path_x"];
  const auto& y = j["previous_path_y"]; 
  if(x.size() != y.size())
    throw std::runtime_error("previous_path_x/previous_path_y size mismatch");
  return to_vector(x, y);
}

Point destination(const json& j) {
  return Point{
    j["end_path_s"].get<float>(),
    j["end_path_d"].get<float>()
  };
}

Fusion::value_type vehicle(const std::vector<float>& v) {
  if(v.size() < 7) {
    throw std::runtime_error("unexpected fusion object");
  }
  
  return Fusion::value_type{
    v[0],
    Vehicle{Point{v[1], v[2]}, Point{v[3], v[4]}, Point{v[5], v[6]}}
  };
}
  
Fusion fusion(const json& j) {
  const auto& o = j["sensor_fusion"];
  Fusion f;
  std::transform(begin(o), end(o), std::inserter(f, end(f)), vehicle);
  return f;
}
}

Model Json::operator()(std::string message) const {
  auto j = extract(std::move(message));
  return Model{ego(j), path(j), destination(j), fusion(j)};
}

std::string Json::operator()(Path path) const {
  json j;
  j["next_x"] = extract(path, [](const Point& p) { return p.x; });
  j["next_y"] = extract(path, [](const Point& p) { return p.y; });
  return j.dump();
}
