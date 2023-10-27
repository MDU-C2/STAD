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
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>  
#include <stdio.h>  
#include <string.h>  

#define PORT 8080  

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

/// @brief Bluetooth communication for client device
/// @param remote_connection 
/// @return ?????????
int run_bt_client(std::string remote_connection);

/// @brief Bluetooth communication for server device
/// @param remote_connection 
/// @return ?????????
int run_bt_server();

/// @brief Ethernet communication for client device
/// @param remote_connection 
/// @return /
int run_eth_client(std::string remote_connection);

/// @brief Ethernet communication for server device
/// @param remote_connection 
/// @return /
int run_eth_server(std::string remote_connection);

/// @brief Function to parse an INI-style configuration file
/// @param filename 
/// @return Return type is a map of program configurations
std::map<std::string, std::string> parse_ini_file(const std::string& filename);

#endif