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
#include <queue>
#include <mutex>

#define PORT 1337  
#define EMERGENCY_LANDING_THRESHOLD_MS 1000
#define HELLO_RATE_MS 50
#define QUEUE_SIZE EMERGENCY_LANDING_THRESHOLD_MS/HELLO_RATE_MS * 1.2 
//original values 1000/50*1.2 will make a queue size of 24

std::mutex send_mutex;
std::mutex imu_flag_mutex;

std::queue<long long> message_queue;
int eth_client_socket;
int eth_server_socket;
int bt_client_socket;
int bt_server_socket;
bool imu_flag;

std::map<std::string, std::string> config_data;
std::string device;

enum Information : uint8_t {    
    imu,
    start,    
    land,
    ack    
    //stop,
};

struct Data {
    long long   id;
    Information info;          
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
int run_eth_server();

/// @brief Function to parse an INI-style configuration file
/// @param filename 
/// @return Return type is a map of program configurations
std::map<std::string, std::string> parse_ini_file(const std::string& filename);

#endif