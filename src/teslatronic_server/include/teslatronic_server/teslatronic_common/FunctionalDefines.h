#ifndef FUNCTIONALDEFINES_H_
#define FUNCTIONALDEFINES_H_

#include <functional>

#include "teslatronic_server/teslatronic_common/CommonDefines.h"

using SetEngineStateCb = std::function<void(EngineState)>;

#endif /* FUNCTIONALDEFINES_H_ */
