#include "teslatronic_server/teslatronic_common/Ros2Communicator.h"

#include <iostream>

int32_t Ros2Communicator::init() {
  _executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  if (!_executor) {
    std::cerr << "Error, bad alloc for SingleThreadedExecutor" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

void Ros2Communicator::registerNode(const std::shared_ptr<rclcpp::Node> &node) {
  _executor->add_node(node);
}

void Ros2Communicator::unregisterNode(
    const std::shared_ptr<rclcpp::Node> &node) {
  _executor->remove_node(node);
}

void Ros2Communicator::run() {
  _executor->spin();
}
