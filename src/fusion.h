#pragma once

#include <vector>
#include "util.h"
#include "limits.h"
#include "model.h"

class Map;

Vehicle estimate(const Vehicle& v, Timestamp ts, const Map& map);
std::vector<Vehicle> estimate(const Fusion& f, Timestamp ts, const Map& map);
