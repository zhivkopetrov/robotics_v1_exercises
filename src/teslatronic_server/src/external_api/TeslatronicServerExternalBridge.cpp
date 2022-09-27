#include "teslatronic_server/external_api/TeslatronicServerExternalBridge.h"

#include <iostream>

namespace {
constexpr auto NODE_NAME = "TeslatronicServerExternalBridge";
constexpr auto ENGINE_START_STOP_TOPIC_NAME = "engine_start_stop";
}

TeslatronicServerExternalBridge::TeslatronicServerExternalBridge()
    : Node(NODE_NAME) {

}

int32_t TeslatronicServerExternalBridge::init() {
  constexpr auto queueSize = 10;
  const rclcpp::QoS qos(queueSize);

  _engineStartStopSubscriber = create_subscription<Int32>(
      ENGINE_START_STOP_TOPIC_NAME, qos,
      std::bind(&TeslatronicServerExternalBridge::onEngineStartStopMsg, this,
          std::placeholders::_1));

  return EXIT_SUCCESS;
}

void TeslatronicServerExternalBridge::onEngineStartStopMsg(
    const std::shared_ptr<Int32> msg) {
  std::cout << "Engine state received: " << msg->data << std::endl;
}
