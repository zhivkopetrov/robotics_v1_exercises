#ifndef TESLATRONICCLIENTEXTERNALBRIDGE_H_
#define TESLATRONICCLIENTEXTERNALBRIDGE_H_

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/int32.hpp>

class TeslatronicClientExternalBridge : public rclcpp::Node {
public:
  TeslatronicClientExternalBridge();

  int32_t init();

  void run();

private:
  using Int32 = std_msgs::msg::Int32;

  std::shared_ptr<rclcpp::Publisher<Int32>> _engineStartStopPublisher;
};

#endif /* TESLATRONICCLIENTEXTERNALBRIDGE_H_ */
