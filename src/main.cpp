#include <iostream>
#include <memory>
#include <stdexcept>

#include <uWS/uWS.h>

#include "config.h"
#include "application.h"
#include "protocol.h"
#include "compose.h"
#include "planner.h"
#include "io.h"

namespace {
Compose<Planner>
pipeline(Config ) {
  // TODO: support reconfiguration
  return {
    Planner{},
/*    MPC{c.depth, c.dt, *stream},
    Delay{c.delay},
    Rotate{}*/
  }; // applied from right to left
}

using WSApplication = Application<WSProtocol, Json, Json,
                                  std::result_of<decltype(&pipeline)(Config)>::type>;
  
void run(Config c) {
  std::cout << "Using the following config: " << c << "\n";
  std::unique_ptr<WSApplication> app;

  uWS::Hub h;
  h.onMessage([&app, c](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode) {
    try {
      auto message = std::string(data, length);
      std::cerr << ">>>\n" << message << "\n";
      auto response = app->ProcessMessage(std::move(message));
      ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
    } catch(std::runtime_error& e) {
      std::cerr << "Error while processing message: " << e.what() << std::endl;
    }
  });

  h.onConnection([&app, c](uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest) {
      app.reset(new WSApplication(pipeline(c)));
  });

  h.onDisconnection([&app](uWS::WebSocket<uWS::SERVER>, int, char*, size_t) {
    app.reset();
  });

  if (h.listen(c.port)) {
    std::cout << "Listening to port " << c.port << std::endl;
    h.run();
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
  }
}
}

int main(int argc, char* argv[]) {
  try {    
    run(Config(argc, argv));
  } catch(std::exception& e) {
    std::cerr << "Program aborted with error: " << e.what() << std::endl;
    exit(1);
  }
  return 0;
}
