#ifndef FUNCTIONALDEFINES_H_
#define FUNCTIONALDEFINES_H_

#include <functional>
#include <vector>

#include "teslatronic_server/teslatronic_common/CommonDefines.h"

using SetEngineStateCb = std::function<void(EngineState)>;
using GetMapDescrCb = std::function<const MapDescription&()>;
using GetBatteryInfoCb = std::function<const BatteryInfo&()>;

#endif /* FUNCTIONALDEFINES_H_ */
