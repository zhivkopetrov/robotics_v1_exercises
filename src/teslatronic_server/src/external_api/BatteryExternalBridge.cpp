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

  _chargeBatteryActionServer = rclcpp_action::create_server<ChargeBattery>(this,
      CHARGE_BATTERY_ACTION_NAME,
      std::bind(&BatteryExternalBridge::handleChargeBatteryGoal, this, _1, _2),
      std::bind(&BatteryExternalBridge::handleChargeBatteryCancel, this, _1),
      std::bind(&BatteryExternalBridge::handleChargeBatteryAccepted, this, _1));

  _batteryDepleteHeadTimer = create_wall_timer(1s, [this]() {
    std::lock_guard<std::mutex> lock(_batteryMutex);
    _outInterface.depleteHeatCb();
  });

  return EXIT_SUCCESS;
}

void BatteryExternalBridge::handleBatterInfoQueryService(
    [[maybe_unused]]const std::shared_ptr<QueryBatteryInfo::Request> request,
    std::shared_ptr<QueryBatteryInfo::Response> response) {
  std::lock_guard<std::mutex> lock(_batteryMutex);
  const BatteryInfo &data = _outInterface.getBatteryInfoCb();
  fillBatteryInfo(data, response->battery_info);
}

rclcpp_action::GoalResponse BatteryExternalBridge::handleChargeBatteryGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ChargeBattery::Goal> goal) {
  std::cout << "Received goal request with charge_turns: " << goal->charge_turns
            << ", uuid: " << rclcpp_action::to_string(uuid) << std::endl;

  _outInterface.setChargeStateCb(BatteryChargeState::ACTIVE);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BatteryExternalBridge::handleChargeBatteryCancel(
    const std::shared_ptr<GoalHandleChargeBattery> goalHandle) {
  std::cout << "Canceling goal with UUID: "
            << rclcpp_action::to_string(goalHandle->get_goal_id()) << std::endl;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BatteryExternalBridge::handleChargeBatteryAccepted(
    const std::shared_ptr<GoalHandleChargeBattery> goalHandle) {
  std::thread { std::bind(&BatteryExternalBridge::executeGoal, this, _1),
      goalHandle }.detach();
}

void BatteryExternalBridge::executeGoal(
    const std::shared_ptr<GoalHandleChargeBattery> goalHandle) {
  using namespace std::literals;

  const auto goal = goalHandle->get_goal();
  auto feedback = std::make_shared<ChargeBattery::Feedback>();
  auto result = std::make_shared<ChargeBattery::Result>();
  feedback->charge_turns_left = goal->charge_turns;

  std::unique_lock<std::mutex> lock(_batteryMutex);
  const BatteryInfo &batteryData = _outInterface.getBatteryInfoCb();

  while (0 < feedback->charge_turns_left) {
    // Check if there is a cancel request
    if (goalHandle->is_canceling()) {
      result->success = true;

      goalHandle->canceled(result);
      std::cout << "Goal with UUID: "
                << rclcpp_action::to_string(goalHandle->get_goal_id())
                << " was cancelled" << std::endl;

      fillBatteryInfo(batteryData, result->battery_info);
      _outInterface.setChargeStateCb(BatteryChargeState::IDLE);
      return;
    }

    const bool success = _outInterface.chargeBatterySingleTurnCb();
    if (!success) {
      result->success = false;
      result->error_reason = "Goal with UUID: ";
      result->error_reason.append(
          rclcpp_action::to_string(goalHandle->get_goal_id())).append(
          " was cancelled due to maximum heat reached");
      std::cout << result->error_reason << std::endl;

      fillBatteryInfo(batteryData, result->battery_info);
      goalHandle->succeed(result);
      _outInterface.setChargeStateCb(BatteryChargeState::IDLE);
      return;
    }

    --feedback->charge_turns_left;
    fillBatteryInfo(batteryData, feedback->battery_info);
    goalHandle->publish_feedback(feedback);

    lock.unlock();
    std::this_thread::sleep_for(500ms);
    lock.lock();
  }

  fillBatteryInfo(batteryData, result->battery_info);
  result->success = true;
  goalHandle->succeed(result);
  _outInterface.setChargeStateCb(BatteryChargeState::IDLE);
}

