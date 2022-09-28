#ifndef TESLATRONICCLIENTEXTERNALBRIDGE_H_
#define TESLATRONICCLIENTEXTERNALBRIDGE_H_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/client.hpp>
#include <teslatronic_interfaces/msg/engine_start_stop.hpp>
#include <teslatronic_interfaces/srv/query_map.hpp>

class TeslatronicClientExternalBridge : public rclcpp::Node {
public:
  TeslatronicClientExternalBridge();

  int32_t init();

  void run();

private:
  void queryMap();

  using EngineStartStop = teslatronic_interfaces::msg::EngineStartStop;
  using QueryMap = teslatronic_interfaces::srv::QueryMap;

  std::shared_ptr<rclcpp::Publisher<EngineStartStop>> _engineStartStopPublisher;
  std::shared_ptr<rclcpp::Client<QueryMap>> _mapQueryClient;
};

#endif /* TESLATRONICCLIENTEXTERNALBRIDGE_H_ */
