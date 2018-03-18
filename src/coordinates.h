#pragma once

#include "map.h"
#include "point.h"

namespace frenet {
Point to(Heading h, const Map& m);
Point to(Point p, float h, const Map& m);
Point to(Point p, Point prev, const Map& m);
Point from(Point p, const Map& m);
}

namespace local {
Point to(Point p, Heading origin);
Point from(Point p, Heading origin);
}
