#pragma once

#include <string>

template<typename Protocol, typename Deserializer, typename Serializer, typename Pipeline>
class Application {
public:
  explicit Application(Pipeline pipeline) :
    pipeline_(std::move(pipeline))
  {}
  
  std::string ProcessMessage(std::string message) {
    auto request = Protocol::getPayload(std::move(message));
    if(request.empty()) return Protocol::formatResponse();

    auto input = deserializer_(std::move(request));
    auto output = pipeline_(std::move(input));
    auto response = serializer_(std::move(output));
    return Protocol::formatResponse(std::move(response));
  }

private:
  Deserializer deserializer_;
  Serializer serializer_;
  Pipeline pipeline_;
};
