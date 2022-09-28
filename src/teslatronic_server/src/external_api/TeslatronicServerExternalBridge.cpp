#include "teslatronic_server/external_api/TeslatronicServerExternalBridge.h"

#include "teslatronic_server/teslatronic_common/TeslatronicTopics.h"
#include "teslatronic_server/teslatronic_common/MessageHelpers.h"

namespace {
constexpr auto NODE_NAME = "TeslatronicServerExternalBridge";
}

TeslatronicServerExternalBridge::TeslatronicServerExternalBridge()
    : Node(NODE_NAME) {

}

int32_t TeslatronicServerExternalBridge::init(
    const TeslatronicServerExternalBridgeOutInterface &outInterface) {
  using namespace std::placeholders;

  _outInterface = outInterface;
  if (nullptr == _outInterface.setEngineStateCb) {
    std::cerr << "Error, nullptr provided for SetEngineStateCb" << std::endl;
    return EXIT_FAILURE;
  }
  if (nullptr == _outInterface.getMapDescrCb) {
    std::cerr << "Error, nullptr provided for GetMapDescrCb" << std::endl;
    return EXIT_FAILURE;
  }

  constexpr auto queueSize = 10;
  const rclcpp::QoS qos(queueSize);

  _engineStartStopSubscriber = create_subscription<EngineStartStop>(
      ENGINE_START_STOP_TOPIC_NAME, qos,
      std::bind(&TeslatronicServerExternalBridge::onEngineStartStopMsg, this,
          _1));

  _mapQueryService = create_service<QueryMap>(QUERY_MAP_SERVICE_NAME,
      std::bind(&TeslatronicServerExternalBridge::handleMapQueryService, this,
          _1, _2));

  return EXIT_SUCCESS;
}

void TeslatronicServerExternalBridge::onEngineStartStopMsg(
    const std::shared_ptr<EngineStartStop> msg) {
  const EngineState state = toEngineState(msg->state);
  _outInterface.setEngineStateCb(state);
}

void TeslatronicServerExternalBridge::handleMapQueryService(
    [[maybe_unused]]const std::shared_ptr<QueryMap::Request> request,
    std::shared_ptr<QueryMap::Response> response) {
  const auto &mapDescr = _outInterface.getMapDescrCb();
  response->map.rows = mapDescr.rows;
  response->map.cols = mapDescr.cols;
  response->map.data = mapDescr.data;
}
