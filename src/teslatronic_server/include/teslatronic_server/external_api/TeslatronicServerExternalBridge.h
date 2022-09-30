#ifndef TESLATRONICSERVEREXTERNALBRIDGE_H_
#define TESLATRONICSERVEREXTERNALBRIDGE_H_

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/service.hpp>
#include <teslatronic_interfaces/msg/engine_start_stop.hpp>
#include <teslatronic_interfaces/srv/query_map.hpp>

#include "teslatronic_server/teslatronic_common/FunctionalDefines.h"

struct TeslatronicServerExternalBridgeOutInterface {
  SetEngineStateCb setEngineStateCb;
  GetMapDescrCb getMapDescrCb;
};

class TeslatronicServerExternalBridge: public rclcpp::Node {
public:
  TeslatronicServerExternalBridge();

  int32_t init(const TeslatronicServerExternalBridgeOutInterface &outInterface);

private:
  using EngineStartStop = teslatronic_interfaces::msg::EngineStartStop;
  using QueryMap = teslatronic_interfaces::srv::QueryMap;

  void onEngineStartStopMsg(const std::shared_ptr<EngineStartStop> msg);

  void handleMapQueryService(const std::shared_ptr<QueryMap::Request> request,
                             std::shared_ptr<QueryMap::Response> response);

  std::shared_ptr<rclcpp::Subscription<EngineStartStop>> _engineStartStopSubscriber;
  std::shared_ptr<rclcpp::Service<QueryMap>> _mapQueryService;

  TeslatronicServerExternalBridgeOutInterface _outInterface;
};

#endif /* TESLATRONICSERVEREXTERNALBRIDGE_H_ */
