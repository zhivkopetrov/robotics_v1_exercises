#ifndef APPLICATION_H_
#define APPLICATION_H_

#include "teslatronic_server/external_api/TeslatronicServerExternalBridge.h"
#include "teslatronic_server/external_api/BatteryExternalBridge.h"
#include "teslatronic_server/teslatronic_common/Ros2Communicator.h"
#include "teslatronic_server/core/CarControlUnit.h"
#include "teslatronic_server/core/Map.h"
#include "teslatronic_server/core/Battery.h"

struct ApplicationConfig {
  int32_t mapRows{};
  int32_t mapCols{};
  BatteryInfo batteryCfg;
};

class Application {
public:
  ~Application() noexcept;

  int32_t init(const ApplicationConfig& cfg);

  void run();

private:
  int32_t initServerExternalBridge();
  int32_t initBatteryExternalBridge();

  CarControlUnit _carControlUnit;
  Map _map;
  Battery _battery;
  std::shared_ptr<TeslatronicServerExternalBridge> _serverExternalBridge;
  std::shared_ptr<BatteryExternalBridge> _batteryExternalBridge;
  Ros2Communicator _communicator;
};

#endif /* APPLICATION_H_ */
