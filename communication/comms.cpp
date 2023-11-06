#include "comms.h"

std::map<Information, std::string> informationMap{{imu, "imu"},
                                                  {start, "start"},
                                                  //{stop, "stop"},
                                                  {land, "land"}};

void send_bt_message(Data send_data) {
  int socket = (device == "itx") ? bt_server_socket : bt_client_socket;
  DEBUG_MSG("Device name is ITX?  " << (device == "itx"));
  DEBUG_MSG("socket:  " << socket);
  DEBUG_MSG("sned_data.id:   " << send_data.id);
  DEBUG_MSG("sned_data.info:   " << send_data.info);
  DEBUG_MSG("sned_data.yaw:   " << send_data.imu_data.yaw);
  DEBUG_MSG("sned_data.pitch:   " << send_data.imu_data.pitch);
  DEBUG_MSG("sned_data.roll:   " << send_data.imu_data.roll);
  char buffer[sizeof(struct Data)];
  memcpy(buffer, &send_data, sizeof(struct Data));
  std::unique_lock<std::mutex> lock(send_mutex);
  DEBUG_MSG("Data has been sent from " << device);
  send(socket, buffer, sizeof(struct Data), 0);
  lock.unlock();
}

Data form_ack_data(long long id) {
  struct Data send_data = {};
  send_data.info = ack;
  send_data.id = id;
  return send_data;
}

Data form_imu_data(long long message_id) {
  vec3f yaw_pitch_roll;
  CompositeData cd = ez->getNextData();

  if (!cd.hasYawPitchRoll())
    std::cout << "YPR Unavailable." << std::endl;
  else
    yaw_pitch_roll = cd.yawPitchRoll();

  struct Data send_data = {};
  send_data.id = message_id;
  send_data.info = imu;
  send_data.imu_data.yaw = yaw_pitch_roll.x;
  send_data.imu_data.pitch = yaw_pitch_roll.y;
  send_data.imu_data.roll = yaw_pitch_roll.z;
  return send_data;
}
Data form_start_data() {
  struct Data send_data = {};
  send_data.info = start;
  return send_data;
}
Data form_land_data() {
  struct Data send_data = {};
  send_data.info = land;
  return send_data;
}
void itx_bt_message_handler(Data rcvd_data) {
  DEBUG_MSG("received ack for the message id: " << rcvd_data.id);
  while (!message_queue.empty()) {
    if (message_queue.front() == rcvd_data.id) {
      DEBUG_MSG("id was equal pop");
      message_queue.pop();
      break;
    } else {
      DEBUG_MSG("pop to find the message.");
      message_queue.pop();
    }
  }
}
void drone_message_handler(Data rcvd_data) {
  if (rcvd_data.info == imu) {
    DEBUG_MSG("Message received in client with id: " << rcvd_data.id);
    send_bt_message(form_ack_data(rcvd_data.id));
    DEBUG_MSG("Ack for message with this ID sent: " << rcvd_data.id);
    std::unique_lock<std::mutex> lock(imu_flag_mutex);
    imu_flag = true;
    lock.unlock();
  } else if (rcvd_data.info == start) {
    send_bt_message(form_start_data());
  } else if (rcvd_data.info == land) {
    std::cout << "Emergency land because queue is too large in ITX \n";
    // Call Elon's emergency land function;
    // Elon, BE WARY, when the ITX queue size is too large, it will CONTINUOUSLY
    // SEND LAND MSG. Please handle it. For example, have a flag that resets
    // when the drone has landed or something.
  } else {
    std::cout << "Invalid Information.\n";
  }
}
void message_handler(Data rcvd_data) {
  // DEBUG_MSG("Info: " << informationMap[rcvd_data.info] );
  if (device == "itx") {
    DEBUG_MSG("Message Handler in itx.");
    itx_bt_message_handler(rcvd_data);
  } else if (device == "drone") {
    DEBUG_MSG("Message Handler in drone.");
    drone_message_handler(rcvd_data);
  } else {
    std::cout << "Invalid device... exiting.\n";
  }
}

void receive_data(int client) {
  char buf[1024];
  int bytes_read{};

  while (true) {
    memset(buf, 0, sizeof(buf));
    bytes_read = read(client, buf, sizeof(buf));
    if (bytes_read > 0) {
      struct Data rcvd_data;
      ////////////////remove to test/////////////////
      // bytes_read = read(client, buf, sizeof(buf));
      DEBUG_MSG("Message received.");
      memcpy(&rcvd_data, buf, sizeof(struct Data));
      message_handler(rcvd_data);
    }
    memset(buf, 0, sizeof(buf));
  }
}

void check_connection() {
  auto start = std::chrono::high_resolution_clock::now();
  while (true) {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    if (duration.count() >= EMERGENCY_LANDING_THRESHOLD_MS) {
      DEBUG_MSG(
          "EMERGENCY_LANDING_THRESHOLD_MS: " << EMERGENCY_LANDING_THRESHOLD_MS);
      DEBUG_MSG("duration.count(): " << duration.count());
      std::cout << "Emergency land because connection timeout \n";
      // call Elons emergency land function
      return;
    }
    std::unique_lock<std::mutex> lock(imu_flag_mutex);
    if (imu_flag == true)  // reset counter
    {
      imu_flag = false;
      start = std::chrono::high_resolution_clock::now();
    }
    lock.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(HELLO_RATE_MS));
  }
}

int run_bt_server() {
  int sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
  sockaddr_rc loc_addr = {0};
  loc_addr.rc_family = AF_BLUETOOTH;
  bdaddr_t bdaddr = {0, 0, 0, 0, 0, 0};  // Initialize a bdaddr_t variable
  loc_addr.rc_bdaddr = bdaddr;
  loc_addr.rc_channel = 3;  // RFCOMM channel to use (e.g., 1).

  const std::string SensorPort =
      "/dev/ttyUSB0";  // Linux format for virtual (USB) serial port.
  const uint32_t SensorBaudrate = 115200;
  ez = EzAsyncData::connect(SensorPort, SensorBaudrate);

  long long message_id = 0;

  bind(sock, (struct sockaddr*)&loc_addr, sizeof(loc_addr));
  listen(sock, 3);  // Allow one connection at a time.

  std::cout << "Waiting for a BT connection...\n";

  sockaddr_rc rem_addr = {0};
  socklen_t opt = sizeof(rem_addr);
  int client = accept(sock, (struct sockaddr*)&rem_addr, &opt);
  std::cout << "BT connection accepted\n";
  bt_server_initial_socket = sock;
  bt_server_socket = client;
  DEBUG_MSG("socket when connected to client:  " << bt_server_socket);
  std::thread receiveThread(receive_data, client);

  while (true) {
    message_id += 1;
    if (message_queue.size() > QUEUE_SIZE) {
      std::cout << "Sending emergency landing... Queue size too big. \n";
      send_bt_message(form_land_data());
    }
    send_bt_message(form_imu_data(message_id));
    message_queue.push(message_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(HELLO_RATE_MS));
  }

  receiveThread.join();
  ez->disconnect();
  delete ez;
  close(bt_server_socket);
  close(bt_server_initial_socket);
  return 0;
}

int run_bt_client(std::string remote_connection) {
  int sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
  int status;
  sockaddr_rc server_addr = {0};
  server_addr.rc_family = AF_BLUETOOTH;
  const char* dest = remote_connection.c_str();

  str2ba("00:1A:7D:DA:71:13",
         &server_addr.rc_bdaddr);  // Replace with the server's address.
  server_addr.rc_channel = 3;  // RFCOMM channel to use (must match the server).

  for (int i = 0; i < 10; i++) {
    std::cout << "Connecting... Attempt: " << i + 1 << "\n";
    status = connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (status == 0) {
      std::cout << "Successfully connected to server. \n";
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  }

  if (status < 0) {
    perror("Connection failed");
    close(sock);
  }
  std::thread receiveThread(receive_data, sock);
  std::thread connectionCheckerThread(check_connection);
  bt_client_socket = sock;
  receiveThread.join();
  connectionCheckerThread.join();
  close(sock);
  return 0;
}

int run_eth_server() {
  int server_socket{socket(AF_INET, SOCK_STREAM, 0)};
  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT);
  server_addr.sin_addr.s_addr =
      INADDR_ANY;  // Listen on all available interfaces

  bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
  listen(server_socket, 1);
  std::cout << "Waiting for an Ethernet connection...\n";

  sockaddr_in clientAddr;
  socklen_t clientAddrLen = sizeof(clientAddr);
  int client =
      accept(server_socket, (struct sockaddr*)&clientAddr, &clientAddrLen);
  std::cout << "Ethernet connection accepted\n";

  std::thread receiveThread(receive_data, client);
  eth_server_socket = client;
  eth_server_initial_socket = server_socket;
  receiveThread.join();
  close(eth_server_socket);
  close(eth_server_initial_socket);
  return 0;
}

int run_eth_client(std::string remote_connection) {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  int status;

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT);
  server_addr.sin_addr.s_addr = inet_addr(remote_connection.c_str());

  for (int i = 0; i < 5; i++) {
    std::cout << "Connecting... Attempt: " << i + 1 << "\n";
    status = connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (status == 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  }

  if (status < 0) {
    perror("Connection failed");
    close(sock);
    return 0;
  }
  eth_client_socket = sock;
  std::thread receiveThread(receive_data, sock);
  receiveThread.join();
  close(eth_client_socket);
  return 0;
}

void sigintHandler(int signal) {
  std::cout << "Ctrl+C detected." << std::endl;
  if (device == "itx") {
    // close(eth_server_socket);
    // close(eth_server_initial_socket);
    std::cout << "Disconnecting USB connection." << std::endl;
    ez->disconnect();
    delete ez;
    std::cout << "Killing bluetooth server socket connections." << std::endl;
    close(bt_server_socket);
    close(bt_server_initial_socket);
  } else if (device == "drone") {
    std::cout << "Killing bluetooth client socket connections." << std::endl;
    close(bt_client_socket);
  } else if (device == "px2") {
    std::cout << "Killing ethernet client socket connections." << std::endl;

    close(eth_client_socket);
  } else {
    std::cout << "Invalid device... exiting.\n";
  }
  // Terminate program
  exit(signal);
}

std::map<std::string, std::string> parse_ini_file(const std::string& filename) {
  std::map<std::string, std::string> config;

  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Error: Could not open " << filename << std::endl;
    return config;
  }

  std::string line;
  while (std::getline(file, line)) {
    // Split the line into key and value
    size_t delimiterPos = line.find('=');
    if (delimiterPos != std::string::npos) {
      std::string key = line.substr(0, delimiterPos);
      std::string value = line.substr(delimiterPos + 1);
      config[key] = value;
    }
  }
  return config;
}