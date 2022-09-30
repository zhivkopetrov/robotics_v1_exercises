#include "teslatronic_server/external_api/BatteryExternalBridge.h"

#include "teslatronic_server/teslatronic_common/TeslatronicTopics.h"
#include "teslatronic_server/teslatronic_common/MessageHelpers.h"

namespace {
constexpr auto NODE_NAME = "BatteryExternalBridge";

using BatteryInfoMsgData = teslatronic_interfaces::msg::BatteryInfo;

void fillBatteryInfo(const BatteryInfo &data, BatteryInfoMsgData &outData) {
  outData.batery_model = data.model;
  outData.current_heat = data.currentHeat;
  outData.max_heat = data.maxHeat;
  outData.current_power = data.currentPower;
  outData.max_power = data.maxPower;
}
}

using namespace std::placeholders;
using namespace std::literals;

BatteryExternalBridge::BatteryExternalBridge()
    : Node(NODE_NAME) {

}

int32_t BatteryExternalBridge::init(
    const BatteryExternalBridgeOutInterface &outInterface) {
  _outInterface = outInterface;
  if (nullptr == _outInterface.getBatteryInfoCb) {
    std::cerr << "Error, nullptr provided for GetBatteryInfoCb" << std::endl;
    return EXIT_FAILURE;
  }
  if (nullptr == _outInterface.setChargeStateCb) {
    std::cerr << "Error, nullptr provided for SetChargeStateCb" << std::endl;
    return EXIT_FAILURE;
  }
  if (nullptr == _outInterface.chargeBatterySingleTurnCb) {
    std::cerr << "Error, nullptr provided for ChargeBatterySingleTurnCb"
              << std::endl;
    return EXIT_FAILURE;
  }
  if (nullptr == _outInterface.depleteHeatCb) {
    std::cerr << "Error, nullptr provided for DepleteHeatCb" << std::endl;
    return EXIT_FAILURE;
  }

  constexpr auto queueSize = 10;
  const rclcpp::QoS qos(queueSize);

  _batteryInfoQueryService = create_service<QueryBatteryInfo>(
      QUERY_BATTERY_INFO_SERVICE_NAME,
      std::bind(&BatteryExternalBridge::handleBatterInfoQueryService, this, _1,
          _2));

  _batteryDepleteHeadTimer = create_wall_timer(1s, [this]() {
    _outInterface.depleteHeatCb();
  });

  return EXIT_SUCCESS;
}

void BatteryExternalBridge::handleBatterInfoQueryService(
    [[maybe_unused]]const std::shared_ptr<QueryBatteryInfo::Request> request,
    std::shared_ptr<QueryBatteryInfo::Response> response) {
  const BatteryInfo &data = _outInterface.getBatteryInfoCb();
  fillBatteryInfo(data, response->battery_info);
}
