#include "teslatronic_server/core/CarControlUnit.h"

#include <iostream>

void CarControlUnit::setEngineState(EngineState state) {
  _engineState = state;

  std::cout << "Changing EngineState to: " << static_cast<int32_t>(_engineState)
            << std::endl;
}
