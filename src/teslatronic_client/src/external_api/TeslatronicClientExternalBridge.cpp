#include "teslatronic_client/external_api/TeslatronicClientExternalBridge.h"

#include <thread>
#include <iostream>

namespace {
constexpr auto NODE_NAME = "TeslatronicClientExternalBridge";
constexpr auto ENGINE_START_STOP_TOPIC_NAME = "engine_start_stop";
constexpr auto QUERY_MAP_SERVICE_NAME = "query_map";
constexpr auto QUERY_BATTERY_INFO_SERVICE_NAME = "query_battery_info";

template <typename T>
void waitForService(const T &client) {
  using namespace std::literals;
  while (!client->wait_for_service(1s)) {
    std::cout << "Service: [" << client->get_service_name()
    << "] not available. Waiting for 1s ..." << std::endl;
  }
}
}

TeslatronicClientExternalBridge::TeslatronicClientExternalBridge()
    : Node(NODE_NAME) {

}

int32_t TeslatronicClientExternalBridge::init() {
  constexpr auto queueSize = 10;
  const rclcpp::QoS qos(queueSize);
  _engineStartStopPublisher = create_publisher<EngineStartStop>(
      ENGINE_START_STOP_TOPIC_NAME, qos);

  _mapQueryClient = create_client<QueryMap>(QUERY_MAP_SERVICE_NAME);
  _batteryInfoQueryClient = create_client<QueryBatteryInfo>(
      QUERY_BATTERY_INFO_SERVICE_NAME);

  waitForService(_mapQueryClient);
  waitForService(_batteryInfoQueryClient);

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

    queryMap();
    queryBatteryInfo();
  }
}

void TeslatronicClientExternalBridge::queryMap() {
  std::shared_ptr<QueryMap::Request> request = std::make_shared<
      QueryMap::Request>();

  auto result = _mapQueryClient->async_send_request(request);
  const std::shared_ptr<QueryMap::Response> response = result.get();
  const auto &map = response->map;
  int32_t idx { };

  for (int32_t row = 0; row < map.rows; ++row) {
    for (int32_t col = 0; col < map.cols; ++col) {
      std::cout << map.data[idx];
      ++idx;
    }
    std::cout << '\n';
  }
  std::cout << std::endl;
}

void TeslatronicClientExternalBridge::queryBatteryInfo() {
  std::shared_ptr<QueryBatteryInfo::Request> request = std::make_shared<
      QueryBatteryInfo::Request>();

  auto result = _batteryInfoQueryClient->async_send_request(request);
  const std::shared_ptr<QueryBatteryInfo::Response> response = result.get();
  const auto &data = response->battery_info;
  std::cout << "BatteryInfo\nModel: " << data.batery_model << "\nPower: ("
            << data.current_power << '/' << data.max_power << ")\nHeat: ("
            << data.current_heat << '/' << data.max_heat << ')' << std::endl;
}
