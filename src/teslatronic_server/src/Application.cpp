#include "teslatronic_server/Application.h"

Application::~Application() noexcept {
  _communicator.unregisterNode(_externalBridge);
}

int32_t Application::init(const ApplicationConfig &cfg) {
  using namespace std::placeholders;

  int32_t errCode = _communicator.init();
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "Ros2Commicator::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  TeslatronicServerExternalBridgeOutInterface outInterface;
  outInterface.setEngineStateCb = std::bind(&CarControlUnit::setEngineState,
      &_carControlUnit, _1);
  outInterface.getMapDescrCb = std::bind(&Map::getMapDescr, &_map);

  _externalBridge = std::make_shared<TeslatronicServerExternalBridge>();
  errCode = _externalBridge->init(outInterface);
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "TeslatronicServerExternalBridge::init() failed" << std::endl;
    return EXIT_FAILURE;
  }
  _communicator.registerNode(_externalBridge);

  errCode = _map.init(cfg.mapRows, cfg.mapCols);
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "Map::init() failed" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

void Application::run() {
  _communicator.run();
}
