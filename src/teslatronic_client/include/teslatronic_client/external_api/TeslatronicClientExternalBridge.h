#ifndef TESLATRONICCLIENTEXTERNALBRIDGE_H_
#define TESLATRONICCLIENTEXTERNALBRIDGE_H_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <teslatronic_interfaces/msg/engine_start_stop.hpp>

class TeslatronicClientExternalBridge : public rclcpp::Node {
public:
  TeslatronicClientExternalBridge();

  int32_t init();

  void run();

private:
  using EngineStartStop = teslatronic_interfaces::msg::EngineStartStop;

  std::shared_ptr<rclcpp::Publisher<EngineStartStop>> _engineStartStopPublisher;
};

#endif /* TESLATRONICCLIENTEXTERNALBRIDGE_H_ */
