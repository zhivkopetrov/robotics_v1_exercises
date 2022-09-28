#include "teslatronic_server/core/Battery.h"

int32_t Battery::init(const BatteryInfo& cfg) {
  _data = cfg;
  return EXIT_SUCCESS;
}

const BatteryInfo& Battery::getBatteryInfo() const {
  return _data;
}
