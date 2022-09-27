#ifndef TESLATRONICSERVEREXTERNALBRIDGE_H_
#define TESLATRONICSERVEREXTERNALBRIDGE_H_

#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/int32.hpp>

class TeslatronicServerExternalBridge : public rclcpp::Node {
public:
  TeslatronicServerExternalBridge();

  int32_t init();

private:
  using Int32 = std_msgs::msg::Int32;

  void onEngineStartStopMsg(const std::shared_ptr<Int32> msg);

  std::shared_ptr<rclcpp::Subscription<Int32>> _engineStartStopSubscriber;
};

#endif /* TESLATRONICSERVEREXTERNALBRIDGE_H_ */
