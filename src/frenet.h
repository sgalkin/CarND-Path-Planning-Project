#pragma once

#include "map.h"
#include "point.h"
/*
std::size_t ClosestWaypoint(float x, float y,
                            const std::vector<float> &maps_x,
                            const std::vector<float> &maps_y);

std::size_t NextWaypoint(float x, float y, float theta,
                         const std::vector<float> &maps_x,
                         const std::vector<float> &maps_y);
*/
namespace to {
Point frenet(Heading h, const Map& m);
Point xy(Point p, const Map& m);
}
