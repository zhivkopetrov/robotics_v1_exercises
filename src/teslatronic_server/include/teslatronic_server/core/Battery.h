#ifndef BATTERY_H_
#define BATTERY_H_

#include "teslatronic_server/teslatronic_common/CommonDefines.h"

struct BatteryConfig {
  BatteryInfo data;
  int32_t powerGainPerTurn { };
  int32_t heatGainPerTurn { };
  int32_t heatDepletionRate { };
};

class Battery {
public:
  int32_t init(const BatteryConfig &cfg);
  const BatteryInfo& getBatteryInfo() const;
  void setChargeState(BatteryChargeState state);
  bool chargeSingleTurn();
  void depleteHeat();

private:
  BatteryInfo _data;
  int32_t _powerGainPerTurn { };
  int32_t _heatGainPerTurn { };
  int32_t _heatDepletionRate { };

  BatteryChargeState _chargeState = BatteryChargeState::IDLE;
};

#endif /* BATTERY_H_ */
