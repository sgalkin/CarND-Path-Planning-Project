#pragma once

#include "map.h"
#include "point.h"

namespace to {
Point frenet(Heading h, const Map& m);
Point xy(Point p, const Map& m);
}
