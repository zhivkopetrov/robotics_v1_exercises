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
  _engineStartStopPublisher = create_publisher<Int32>(
      ENGINE_START_STOP_TOPIC_NAME, qos);

  return EXIT_SUCCESS;
}

void TeslatronicClientExternalBridge::run() {
  using namespace std::literals;

  constexpr int32_t retries = 10;
  bool engineStarted = false;
  Int32 msg;

  for (int32_t i = 0; i < retries; ++i) {
    engineStarted = !engineStarted;
    msg.data = static_cast<int32_t>(engineStarted);
    _engineStartStopPublisher->publish(msg);
    std::this_thread::sleep_for(1s);
  }
}
