#ifndef COMMS_H
#define COMMS_H
#define CONFIG "config.cfg"

#include <string>
#include <map>
#include <iostream>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <unistd.h>
#include <thread>
#include <fstream>

enum Information : uint8_t {
    ping,
    imu,
    start,
    stop,
    land
};

enum Location : uint8_t {
    px2,
    itx,
    drone
};

struct Data {
    Information info;  
    Location    src;
    Location    dest;
    int         imu_data_1;
    int         imu_data_2;
    float       imu_data_3;
    float       imu_data_4;
};

int run_bt_client(std::string remote_connection);
int run_bt_server();
std::map<std::string, std::string> parse_ini_file(const std::string& filename);


#endif