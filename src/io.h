#pragma once

#include <string>
#include "model.h"

struct Json {
  Model operator()(std::string json) const;
  std::string operator()(Path path) const;
};
