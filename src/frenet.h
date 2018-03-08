#pragma once

#include "map.h"
#include "point.h"

namespace frenet {
Point to(Heading h, const Map& m);
Point from(Point p, const Map& m);
}
