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
#include <climits>
#include "vn/ezasyncdata.h"
#include "vn/thread.h"

#define PORT 1337  
#define EMERGENCY_LANDING_THRESHOLD_MS 1000
#define HELLO_RATE_MS 50
#define QUEUE_SIZE EMERGENCY_LANDING_THRESHOLD_MS/HELLO_RATE_MS * 1.2 
//original values 1000/50*1.2 will make a queue size of 24

std::map<std::string, std::string> parse_ini_file(const std::string& filename);
inline std::map<std::string, std::string> config_data;
inline std::queue<long long> message_queue;
inline std::mutex send_mutex;
inline std::mutex imu_flag_mutex;
inline std::string device;
inline bool imu_flag;
inline int eth_client_socket;
inline int eth_server_socket;
inline int bt_client_socket;
inline int bt_server_socket;

using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

enum Information : uint8_t {    
    imu,
    start,    
    land,
    ack    
    //stop,
};

struct DataIMU
{
    float yaw;
    float pitch;
    float roll;
};

struct Data {
    long long   id;
    Information info;          
    DataIMU     imu_data;
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

#endif