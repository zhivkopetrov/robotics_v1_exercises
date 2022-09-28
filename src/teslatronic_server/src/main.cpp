#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "teslatronic_server/Application.h"

static ApplicationConfig generateAppConfig() {
  ApplicationConfig cfg;
  cfg.mapRows = 5;
  cfg.mapCols = 8;

  return cfg;
}

int32_t main(int32_t argc, char *argv[]) {
  rclcpp::InitOptions initOptions;
  initOptions.shutdown_on_sigint = false;
  rclcpp::init(argc, argv);

  const auto appCfg = generateAppConfig();

  Application app;
  const int32_t errCode = app.init(appCfg);
  if (EXIT_SUCCESS != errCode) {
    std::cerr << "initApp() failed" << std::endl;
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
