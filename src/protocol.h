#pragma once

#include <string>

struct WSProtocol {
  static std::string getPayload(std::string request);

  static std::string formatResponse();
  static std::string formatResponse(std::string message);
};
