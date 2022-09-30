#ifndef BATTERYEXTERNALBRIDGE_H_
#define BATTERYEXTERNALBRIDGE_H_

#include <mutex>

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include "rclcpp/timer.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <teslatronic_interfaces/srv/query_battery_info.hpp>
#include <teslatronic_interfaces/action/charge_battery.hpp>

#include "teslatronic_server/teslatronic_common/FunctionalDefines.h"

struct BatteryExternalBridgeOutInterface {
  GetBatteryInfoCb getBatteryInfoCb;
  SetChargeStateCb setChargeStateCb;
  ChargeBatterySingleTurnCb chargeBatterySingleTurnCb;
  DepleteHeatCb depleteHeatCb;
};

class BatteryExternalBridge: public rclcpp::Node {
public:
  BatteryExternalBridge();

  int32_t init(const BatteryExternalBridgeOutInterface &outInterface);

private:
  using QueryBatteryInfo = teslatronic_interfaces::srv::QueryBatteryInfo;
  using ChargeBattery = teslatronic_interfaces::action::ChargeBattery;
  using GoalHandleChargeBattery = rclcpp_action::ServerGoalHandle<ChargeBattery>;

  void handleBatterInfoQueryService(
      const std::shared_ptr<QueryBatteryInfo::Request> request,
      std::shared_ptr<QueryBatteryInfo::Response> response);

  rclcpp_action::GoalResponse handleChargeBatteryGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const ChargeBattery::Goal> goal);

  rclcpp_action::CancelResponse handleChargeBatteryCancel(
      const std::shared_ptr<GoalHandleChargeBattery> goalHandle);

  void handleChargeBatteryAccepted(
      const std::shared_ptr<GoalHandleChargeBattery> goalHandle);

  void executeGoal(const std::shared_ptr<GoalHandleChargeBattery> goalHandle);

  rclcpp::TimerBase::SharedPtr _batteryDepleteHeadTimer;
  std::shared_ptr<rclcpp::Service<QueryBatteryInfo>> _batteryInfoQueryService;
  rclcpp_action::Server<ChargeBattery>::SharedPtr _chargeBatteryActionServer;

  std::mutex _batteryMutex;
  BatteryExternalBridgeOutInterface _outInterface;
};

#endif /* BATTERYEXTERNALBRIDGE_H_ */
