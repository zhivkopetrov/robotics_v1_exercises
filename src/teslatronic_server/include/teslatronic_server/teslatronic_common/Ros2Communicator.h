#ifndef ROS2COMMUNICATOR_H_
#define ROS2COMMUNICATOR_H_

#include <memory>
#include <rclcpp/executors.hpp>

class Ros2Communicator {
public:
  int32_t init();
  void registerNode(const std::shared_ptr<rclcpp::Node>& node);
  void unregisterNode(const std::shared_ptr<rclcpp::Node>& node);
  void run();

private:
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> _executor;
};

#endif /* ROS2COMMUNICATOR_H_ */
