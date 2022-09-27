#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "teslatronic_server/external_api/TeslatronicServerExternalBridge.h"

int32_t main(int32_t argc, char *argv[]) {
  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = false;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TeslatronicServerExternalBridge>();
  const int32_t errCode = node->init();
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "TeslatronicServerExternalBridge::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  //blocking call
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
