#ifndef APPLICATION_H_
#define APPLICATION_H_

#include "teslatronic_server/external_api/TeslatronicServerExternalBridge.h"
#include "teslatronic_server/core/CarControlUnit.h"
#include "teslatronic_server/core/Map.h"

struct ApplicationConfig {
  int32_t mapRows{};
  int32_t mapCols{};
};

class Application {
public:
  int32_t init(const ApplicationConfig& cfg);

  void run();

private:
  CarControlUnit _carControlUnit;
  Map _map;
  std::shared_ptr<TeslatronicServerExternalBridge> _externalBridge;
};

#endif /* APPLICATION_H_ */
