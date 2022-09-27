#include "teslatronic_client/external_api/TeslatronicClientExternalBridge.h"

#include <thread>

namespace {
constexpr auto NODE_NAME = "TeslatronicClientExternalBridge";
constexpr auto ENGINE_START_STOP_TOPIC_NAME = "engine_start_stop";
}

TeslatronicClientExternalBridge::TeslatronicClientExternalBridge()
    : Node(NODE_NAME) {

}

int32_t TeslatronicClientExternalBridge::init() {
  constexpr auto queueSize = 10;
  const rclcpp::QoS qos(queueSize);
  _engineStartStopPublisher = create_publisher<EngineStartStop>(
      ENGINE_START_STOP_TOPIC_NAME, qos);

  return EXIT_SUCCESS;
}

void TeslatronicClientExternalBridge::run() {
  using namespace std::literals;

  constexpr int32_t retries = 10;
  EngineStartStop msg;
  msg.state = EngineStartStop::ENGINE_STARTED;

  for (int32_t i = 0; i < retries; ++i) {
    msg.state =
        (EngineStartStop::ENGINE_STARTED == msg.state) ?
            EngineStartStop::ENGINE_STOPPED : EngineStartStop::ENGINE_STARTED;
    _engineStartStopPublisher->publish(msg);
    std::this_thread::sleep_for(1s);
  }
}
