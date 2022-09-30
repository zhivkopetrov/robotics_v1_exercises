#include "teslatronic_client/external_api/TeslatronicClientExternalBridge.h"

#include <thread>
#include <iostream>
#include <sstream>

namespace {
constexpr auto NODE_NAME = "TeslatronicClientExternalBridge";
constexpr auto ENGINE_START_STOP_TOPIC_NAME = "engine_start_stop";
constexpr auto QUERY_MAP_SERVICE_NAME = "query_map";
constexpr auto QUERY_BATTERY_INFO_SERVICE_NAME = "query_battery_info";
constexpr auto CHARGE_BATTERY_ACTION_NAME = "charge_battery";

using namespace std::literals;
using namespace std::placeholders;

template <typename T>
void waitForService(const T &client) {
  while (!client->wait_for_service(1s)) {
    std::cout << "Service: [" << client->get_service_name()
    << "] not available. Waiting for 1s ..." << std::endl;
  }
}

template <typename T, typename ActionName>
void waitForAction(const T &action, const ActionName &actionName) {
  while (!action->wait_for_action_server(1s)) {
    std::cout << "Action: [" << actionName
    << "] not available. Waiting for 1s ..." << std::endl;
  }
}

using BatteryInfoMsg = teslatronic_interfaces::msg::BatteryInfo;
std::string toString(const BatteryInfoMsg &msg) {
  std::ostringstream ostr;
  ostr << "BatteryInfo\nModel: " << msg.batery_model << "\nPower: ("
       << msg.current_power << '/' << msg.max_power << ")\nHeat: ("
       << msg.current_heat << '/' << msg.max_heat << ")\n";

  return ostr.str();
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
  _chargeBatteryActionClient = rclcpp_action::create_client<ChargeBattery>(this,
      CHARGE_BATTERY_ACTION_NAME);

  waitForService(_mapQueryClient);
  waitForService(_batteryInfoQueryClient);
  waitForAction(_chargeBatteryActionClient, CHARGE_BATTERY_ACTION_NAME);

  return EXIT_SUCCESS;
}

void TeslatronicClientExternalBridge::run() {
  constexpr int32_t retries = 10;
  EngineStartStop msg;
  msg.state = EngineStartStop::ENGINE_STARTED;

  for (int32_t i = 0; i < retries && _isRunning; ++i) {
    msg.state =
        (EngineStartStop::ENGINE_STARTED == msg.state) ?
            EngineStartStop::ENGINE_STOPPED : EngineStartStop::ENGINE_STARTED;
    _engineStartStopPublisher->publish(msg);
    std::this_thread::sleep_for(1s);

    queryMap();
    queryBatteryInfo();

    sendBatteryChargeGoal(5);
    std::this_thread::sleep_for(3s);
  }
}

void TeslatronicClientExternalBridge::shutdown() {
  std::cout << "Initiating shutdown..." << std::endl;
  _isRunning = false;
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
  std::cout << toString(response->battery_info);
}

void TeslatronicClientExternalBridge::sendBatteryChargeGoal(
    int32_t chargeTurns) {
  auto goalMsg = ChargeBattery::Goal();
  goalMsg.charge_turns = chargeTurns;

  std::cout << "Sending ChargeBattery goal with charge_turns: " << chargeTurns
            << std::endl;

  auto sendGoalOptions =
      rclcpp_action::Client<ChargeBattery>::SendGoalOptions();
  sendGoalOptions.goal_response_callback = std::bind(
      &TeslatronicClientExternalBridge::onBatteryChargeGoalResponse, this, _1);
  sendGoalOptions.feedback_callback = std::bind(
      &TeslatronicClientExternalBridge::onBatteryChargeFeedback, this, _1, _2);
  sendGoalOptions.result_callback = std::bind(
      &TeslatronicClientExternalBridge::onBatteryChargeResult, this, _1);

  _chargeBatteryActionClient->async_send_goal(goalMsg, sendGoalOptions);
}

void TeslatronicClientExternalBridge::onBatteryChargeGoalResponse(
    std::shared_future<GoalHandleChargeBattery::SharedPtr> future) {
  const auto goal_handle = future.get();
  if (!goal_handle) {
    std::cerr << "Goal was rejected by server" << std::endl;
  } else {
    std::cout << "Goal accepted by server, waiting for result" << std::endl;
  }
}

void TeslatronicClientExternalBridge::onBatteryChargeFeedback(
    [[maybe_unused]]const GoalHandleChargeBattery::SharedPtr goalHandle,
    const std::shared_ptr<const ChargeBattery::Feedback> feedback) {
  std::cout << "Charging turn left: " << feedback->charge_turns_left << '\n'
            << toString(feedback->battery_info) << std::endl;
}

void TeslatronicClientExternalBridge::onBatteryChargeResult(
    const GoalHandleChargeBattery::WrappedResult &wrappedResult) {
  switch (wrappedResult.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    std::cerr << "Goal was aborted" << std::endl;
    return;
  case rclcpp_action::ResultCode::CANCELED:
    std::cerr << "Goal was aborted" << std::endl;
    return;
  default:
    std::cerr << "Unknown result code: "
              << static_cast<int32_t>(wrappedResult.code) << std::endl;
    return;
  }

  const auto &result = wrappedResult.result;

  std::cout << "Success " << std::boolalpha << result->success
            << ", error_reason: " << result->error_reason << '\n'
            << toString(result->battery_info) << std::endl;
}
