#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "teslatronic_server/external_api/TeslatronicServerExternalBridge.h"
#include "teslatronic_server/core/CarControlUnit.h"

static int32_t initApp(CarControlUnit &carControlUnit,
                       std::shared_ptr<TeslatronicServerExternalBridge> &node) {
  using namespace std::placeholders;

  TeslatronicServerExternalBridgeOutInterface outInterface;
  outInterface.setEngineStateCb = std::bind(&CarControlUnit::setEngineState,
      &carControlUnit, _1);

  const int32_t errCode = node->init(outInterface);
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "TeslatronicServerExternalBridge::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

int32_t main(int32_t argc, char *argv[]) {
  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = false;
  rclcpp::init(argc, argv);

  CarControlUnit carControlUnit;
  auto node = std::make_shared<TeslatronicServerExternalBridge>();
  const int32_t errCode = initApp(carControlUnit, node);
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "initApp() failed" << std::endl;
    return EXIT_FAILURE;
  }

  //blocking call
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
