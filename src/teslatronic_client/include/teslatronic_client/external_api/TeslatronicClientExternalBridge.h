#ifndef TESLATRONICCLIENTEXTERNALBRIDGE_H_
#define TESLATRONICCLIENTEXTERNALBRIDGE_H_

#include <atomic>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <teslatronic_interfaces/msg/engine_start_stop.hpp>
#include <teslatronic_interfaces/srv/query_map.hpp>
#include <teslatronic_interfaces/srv/query_battery_info.hpp>
#include <teslatronic_interfaces/action/charge_battery.hpp>

class TeslatronicClientExternalBridge: public rclcpp::Node {
public:
  TeslatronicClientExternalBridge();

  int32_t init();

  void run();

  void shutdown();

private:
  using EngineStartStop = teslatronic_interfaces::msg::EngineStartStop;
  using QueryMap = teslatronic_interfaces::srv::QueryMap;
  using QueryBatteryInfo = teslatronic_interfaces::srv::QueryBatteryInfo;
  using ChargeBattery = teslatronic_interfaces::action::ChargeBattery;
  using GoalHandleChargeBattery = rclcpp_action::ClientGoalHandle<ChargeBattery>;

  void queryMap();
  void queryBatteryInfo();
  void sendBatteryChargeGoal(int32_t chargeTurns);

  void onBatteryChargeGoalResponse(
      std::shared_future<GoalHandleChargeBattery::SharedPtr> future);
  void onBatteryChargeFeedback(
      const GoalHandleChargeBattery::SharedPtr goalHandle,
      const std::shared_ptr<const ChargeBattery::Feedback> feedback);
  void onBatteryChargeResult(
      const GoalHandleChargeBattery::WrappedResult &result);

  std::shared_ptr<rclcpp::Publisher<EngineStartStop>> _engineStartStopPublisher;
  std::shared_ptr<rclcpp::Client<QueryMap>> _mapQueryClient;
  std::shared_ptr<rclcpp::Client<QueryBatteryInfo>> _batteryInfoQueryClient;
  rclcpp_action::Client<ChargeBattery>::SharedPtr _chargeBatteryActionClient;

  std::atomic<bool> _isRunning = true;
};

#endif /* TESLATRONICCLIENTEXTERNALBRIDGE_H_ */
