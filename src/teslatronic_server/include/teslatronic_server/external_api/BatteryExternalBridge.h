#ifndef BATTERYEXTERNALBRIDGE_H_
#define BATTERYEXTERNALBRIDGE_H_

#include <teslatronic_interfaces/srv/query_battery_info.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include "teslatronic_server/teslatronic_common/FunctionalDefines.h"

struct BatteryExternalBridgeOutInterface {
  GetBatteryInfoCb getBatteryInfoCb;
};

class BatteryExternalBridge: public rclcpp::Node {
public:
  BatteryExternalBridge();

  int32_t init(const BatteryExternalBridgeOutInterface &outInterface);

private:
  using QueryBatteryInfo = teslatronic_interfaces::srv::QueryBatteryInfo;

  void handleBatterInfoQueryService(
      const std::shared_ptr<QueryBatteryInfo::Request> request,
      std::shared_ptr<QueryBatteryInfo::Response> response);

  std::shared_ptr<rclcpp::Service<QueryBatteryInfo>> _batteryInfoQueryService;
  BatteryExternalBridgeOutInterface _outInterface;
};

#endif /* BATTERYEXTERNALBRIDGE_H_ */
