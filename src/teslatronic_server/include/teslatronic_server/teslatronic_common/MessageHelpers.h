#ifndef MESSAGEHELPERS_H_
#define MESSAGEHELPERS_H_

#include <cstdint>

#include "teslatronic_server/teslatronic_common/CommonDefines.h"

EngineState toEngineState(int8_t data);
int8_t getEngineStateField(EngineState state);

#endif /* MESSAGEHELPERS_H_ */
