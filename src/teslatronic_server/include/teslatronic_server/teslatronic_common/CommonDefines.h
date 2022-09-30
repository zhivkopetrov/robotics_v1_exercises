#ifndef TESLATRONICCOMMONDEFINES_H_
#define TESLATRONICCOMMONDEFINES_H_

#include <cstdint>
#include <vector>
#include <string>

enum class EngineState {
  STOPPED, STARTED
};

enum class BatteryChargeState {
  IDLE, ACTIVE
};

using MapDataType = int8_t;

struct MapDescription {
  std::vector<MapDataType> data;
  int32_t rows { };
  int32_t cols { };
};

struct BatteryInfo {
  std::string model;
  int32_t currentPower { };
  int32_t maxPower { };
  int32_t currentHeat { };
  int32_t maxHeat { };
};

#endif /* TESLATRONICCOMMONDEFINES_H_ */
