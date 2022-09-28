#ifndef CARCONTROLUNIT_H_
#define CARCONTROLUNIT_H_

#include "teslatronic_server/teslatronic_common/CommonDefines.h"

class CarControlUnit {
public:
  void setEngineState(EngineState state);

private:
  EngineState _engineState = EngineState::STOPPED;
};

#endif /* CARCONTROLUNIT_H_ */
