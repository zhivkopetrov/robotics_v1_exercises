#include "teslatronic_server/Application.h"

using namespace std::placeholders;

Application::~Application() noexcept {
  _communicator.unregisterNode(_serverExternalBridge);
  _communicator.unregisterNode(_batteryExternalBridge);
}

int32_t Application::init(const ApplicationConfig &cfg) {
  int32_t errCode = _communicator.init();
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "Ros2Commicator::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  errCode = initServerExternalBridge();
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "initServerExternalBridge failed" << std::endl;
    return EXIT_FAILURE;
  }

  errCode = initBatteryExternalBridge();
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "initServerExternalBridge failed" << std::endl;
    return EXIT_FAILURE;
  }

  errCode = _map.init(cfg.mapRows, cfg.mapCols);
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "Map::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  errCode = _battery.init(cfg.batteryCfg);
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "Battery::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

void Application::run() {
  _communicator.run();
}

int32_t Application::initServerExternalBridge() {
  TeslatronicServerExternalBridgeOutInterface outInterface;
  outInterface.setEngineStateCb = std::bind(&CarControlUnit::setEngineState,
      &_carControlUnit, _1);
  outInterface.getMapDescrCb = std::bind(&Map::getMapDescr, &_map);

  _serverExternalBridge = std::make_shared<TeslatronicServerExternalBridge>();
  const int32_t errCode = _serverExternalBridge->init(outInterface);
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "TeslatronicServerExternalBridge::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  _communicator.registerNode(_serverExternalBridge);
  return EXIT_SUCCESS;
}

int32_t Application::initBatteryExternalBridge() {
  BatteryExternalBridgeOutInterface outInterface;
  outInterface.getBatteryInfoCb = std::bind(&Battery::getBatteryInfo,
      &_battery);

  _batteryExternalBridge = std::make_shared<BatteryExternalBridge>();
  const int32_t errCode = _batteryExternalBridge->init(outInterface);
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "BatteryExternalBridge::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  _communicator.registerNode(_batteryExternalBridge);
  return EXIT_SUCCESS;
}
