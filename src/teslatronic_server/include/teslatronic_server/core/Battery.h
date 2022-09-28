#ifndef BATTERY_H_
#define BATTERY_H_

#include "teslatronic_server/teslatronic_common/CommonDefines.h"

class Battery {
public:
  int32_t init(const BatteryInfo& cfg);
  const BatteryInfo& getBatteryInfo() const;

private:
  BatteryInfo _data;
};

#endif /* BATTERY_H_ */
