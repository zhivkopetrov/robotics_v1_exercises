#ifndef TESLATRONICCOMMONDEFINES_H_
#define TESLATRONICCOMMONDEFINES_H_

#include <cstdint>
#include <vector>

enum class EngineState {
  STOPPED, STARTED
};

using MapDataType = int8_t;

struct MapDescription {
  std::vector<MapDataType> data;
  int32_t rows { };
  int32_t cols { };
};

#endif /* TESLATRONICCOMMONDEFINES_H_ */
