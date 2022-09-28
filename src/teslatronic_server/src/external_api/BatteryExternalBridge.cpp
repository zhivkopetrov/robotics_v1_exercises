#include "teslatronic_server/external_api/BatteryExternalBridge.h"

#include "teslatronic_server/teslatronic_common/TeslatronicTopics.h"
#include "teslatronic_server/teslatronic_common/MessageHelpers.h"

namespace {
constexpr auto NODE_NAME = "BatteryExternalBridge";
}

BatteryExternalBridge::BatteryExternalBridge()
    : Node(NODE_NAME) {

}

int32_t BatteryExternalBridge::init(
    const BatteryExternalBridgeOutInterface &outInterface) {
  using namespace std::placeholders;

  _outInterface = outInterface;
  if (nullptr == _outInterface.getBatteryInfoCb) {
    std::cerr << "Error, nullptr provided for GetBatteryInfoCb" << std::endl;
    return EXIT_FAILURE;
  }

  constexpr auto queueSize = 10;
  const rclcpp::QoS qos(queueSize);

  _batteryInfoQueryService = create_service<QueryBatteryInfo>(
      QUERY_BATTERY_INFO_SERVICE_NAME,
      std::bind(&BatteryExternalBridge::handleBatterInfoQueryService, this,
          _1, _2));

  return EXIT_SUCCESS;
}

void BatteryExternalBridge::handleBatterInfoQueryService(
    [[maybe_unused]]const std::shared_ptr<QueryBatteryInfo::Request> request,
    std::shared_ptr<QueryBatteryInfo::Response> response) {
  const BatteryInfo& data = _outInterface.getBatteryInfoCb();
  response->battery_info.batery_model = data.model;
  response->battery_info.current_heat = data.currentHeat;
  response->battery_info.max_heat = data.maxHeat;
  response->battery_info.current_power = data.currentPower;
  response->battery_info.max_power = data.maxPower;
}
