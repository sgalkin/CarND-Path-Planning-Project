#include "protocol.h"

#include <cstring>
#include <stdexcept>

namespace {
bool checkHeader(const std::string& request) {
  // "42" at the start of the message means there's a websocket message event.
  // The 4 signifies a websocket message. The 2 signifies a websocket event
  return request.length() > 2 && strncmp(request.data(), "42", 2) == 0;
}
}

std::string WSProtocol::getPayload(std::string request) {
  if (!checkHeader(request)) {
    throw std::runtime_error("Unexpected message header");
  }

  // Checks if the SocketIO event has JSON data.
  // If there is data the JSON object in string format will be returned,
  // else the empty string "" will be returned.
  auto b1 = request.find_first_of("[");
  auto b2 = request.find_last_of("]");
  if (request.find("null") == std::string::npos &&
      b1 != std::string::npos &&
      b2 != std::string::npos) {
    return request.substr(b1, b2 - b1 + 1);
  }
  return "";
}

std::string WSProtocol::formatResponse() {
  static const std::string manual = "42[\"manual\",{}]";
  return manual;
}

std::string WSProtocol::formatResponse(std::string message) {
  return "42[\"control\"," + std::move(message) + "]";
}
