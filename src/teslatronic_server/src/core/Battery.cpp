#include "teslatronic_server/core/Battery.h"

#include <iostream>

int32_t Battery::init(const BatteryConfig& cfg) {
  _data = cfg.data;
  _powerGainPerTurn = cfg.powerGainPerTurn;
  _heatGainPerTurn = cfg.heatGainPerTurn;
  _heatDepletionRate = cfg.heatDepletionRate;
  return EXIT_SUCCESS;
}

const BatteryInfo& Battery::getBatteryInfo() const {
  return _data;
}

void Battery::setChargeState(BatteryChargeState state) {
  _chargeState = state;
}

bool Battery::chargeSingleTurn() {
  if (BatteryChargeState::ACTIVE != _chargeState) {
    std::cerr << "Battery could not be charged while not in "
        "BatteryChargeState::ACTIVE state" << std::endl;
    return false;
  }

  _data.currentHeat += _heatGainPerTurn;
  if (_data.currentHeat >= _data.maxHeat) {
    _data.currentHeat = _data.maxHeat;
    return false;
  }

  _data.currentPower += _powerGainPerTurn;
  if (_data.currentPower >= _data.maxPower) {
    _data.currentPower = _data.maxPower;
  }

  return true;
}

void Battery::depleteHeat() {
  if (BatteryChargeState::IDLE != _chargeState) {
    return;
  }

  _data.currentHeat -= _heatDepletionRate;
  if (0 > _data.currentHeat) {
    _data.currentHeat = 0;
  }
}
