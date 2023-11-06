#include "comms.h"

int main() {
  imu_flag = false;
  config_data = parse_ini_file(CONFIG);
  device = config_data["device"];
  signal(SIGINT, sigintHandler);
  std::string remote_connection = config_data["remote_connection"];

  if (device == "itx") {
    run_bt_server();
    run_eth_server();
  } else if (device == "drone") {
    run_bt_client(remote_connection);
  } else if (device == "px2") {
    run_eth_client(remote_connection);
  } else {
    std::cout << "Invalid device... exiting.\n";
  }
  return 0;
}