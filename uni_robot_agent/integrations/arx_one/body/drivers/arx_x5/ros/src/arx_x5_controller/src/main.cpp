#include "arx_x5_controller/X5Controller.hpp"

#include <csignal>

void signalHandler(int signum) {
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  std::signal(SIGHUP, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arx::x5::X5Controller>());
  rclcpp::shutdown();
  return 0;
}
