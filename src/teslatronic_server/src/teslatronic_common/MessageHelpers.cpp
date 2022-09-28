#include "teslatronic_server/teslatronic_common/MessageHelpers.h"

#include <iostream>
#include <teslatronic_interfaces/msg/engine_start_stop.hpp>

using EngineStartStop = teslatronic_interfaces::msg::EngineStartStop;

EngineState toEngineState(int8_t data) {
  switch (data) {
  case EngineStartStop::ENGINE_STARTED:
    return EngineState::STARTED;
  case EngineStartStop::ENGINE_STOPPED:
    return EngineState::STOPPED;
  default:
    std::cerr << "Received unsupported EngineStartStop value: " << data
              << std::endl;
    return EngineState::STARTED;
  }
}

int8_t getEngineStateField(EngineState state) {
  return static_cast<int8_t>(state);
}
