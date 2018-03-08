#pragma once

#include "model.h"

class Planner {
public:
  Path operator()(Model m) { return m.path; }
};
