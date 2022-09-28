#ifndef TESLATRONICCLIENTEXTERNALBRIDGE_H_
#define TESLATRONICCLIENTEXTERNALBRIDGE_H_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/client.hpp>
#include <teslatronic_interfaces/msg/engine_start_stop.hpp>
#include <teslatronic_interfaces/srv/query_map.hpp>
#include <teslatronic_interfaces/srv/query_battery_info.hpp>

class TeslatronicClientExternalBridge : public rclcpp::Node {
public:
  TeslatronicClientExternalBridge();

  int32_t init();

  void run();

private:
  void queryMap();
  void queryBatteryInfo();

  using EngineStartStop = teslatronic_interfaces::msg::EngineStartStop;
  using QueryMap = teslatronic_interfaces::srv::QueryMap;
  using QueryBatteryInfo = teslatronic_interfaces::srv::QueryBatteryInfo;

  std::shared_ptr<rclcpp::Publisher<EngineStartStop>> _engineStartStopPublisher;
  std::shared_ptr<rclcpp::Client<QueryMap>> _mapQueryClient;
  std::shared_ptr<rclcpp::Client<QueryBatteryInfo>> _batteryInfoQueryClient;
};

#endif /* TESLATRONICCLIENTEXTERNALBRIDGE_H_ */
