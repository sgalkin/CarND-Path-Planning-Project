#pragma once

#include "path.h"
#include "model.h"
#include "map.h"

class Trajectory {
public:
  Trajectory(Heading ego_heading, float ego_velocity, Path ego_path, const Map* map);

  Point frenet_origin() const { return frenet_; }
  Heading local_origin() const { return local_; }

  Path generate(Path p, size_t size, float center, float v) const;
  
private:
  float velocity_;
  Path base_path_;
  Point frenet_;
  Heading local_;
  const Map* map_;
};
