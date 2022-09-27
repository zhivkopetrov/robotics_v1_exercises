#ifndef TESLATRONICSERVEREXTERNALBRIDGE_H_
#define TESLATRONICSERVEREXTERNALBRIDGE_H_

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <teslatronic_interfaces/msg/engine_start_stop.hpp>

class TeslatronicServerExternalBridge: public rclcpp::Node {
public:
  TeslatronicServerExternalBridge();

  int32_t init();

private:
  using EngineStartStop = teslatronic_interfaces::msg::EngineStartStop;

  void onEngineStartStopMsg(const std::shared_ptr<EngineStartStop> msg);

  std::shared_ptr<rclcpp::Subscription<EngineStartStop>> _engineStartStopSubscriber;
};

#endif /* TESLATRONICSERVEREXTERNALBRIDGE_H_ */
