#include "teslatronic_server/Application.h"

#include <rclcpp/executors.hpp>

int32_t Application::init(const ApplicationConfig &cfg) {
  using namespace std::placeholders;

  TeslatronicServerExternalBridgeOutInterface outInterface;
  outInterface.setEngineStateCb = std::bind(&CarControlUnit::setEngineState,
      &_carControlUnit, _1);

  _externalBridge = std::make_shared<TeslatronicServerExternalBridge>();
  int32_t errCode = _externalBridge->init(outInterface);
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "TeslatronicServerExternalBridge::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  errCode = _map.init(cfg.mapRows, cfg.mapCols);
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "Map::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

void Application::run() {
  rclcpp::spin(_externalBridge);
}
