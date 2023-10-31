#include "comms.h"

// Ugly ugly ugly, but who cares, it works!
std::map<Information, std::string> informationMap{
    {ping, "ping"},
    {imu, "imu"},
    {start, "start"},
    {stop, "stop"},
    {land, "land"}};

std::map<Location, std::string> locationMap{
    {px2, "px2"},
    {itx, "itx"},
    {drone, "drone"}};


void message_handler(Data rcvd_data)
{
    if (device == "itx")
    {
        if (message_queue.size() > 100) //todo: we should change the size 
        { 
            struct Data send_data = {}; 
            send_data.info = land;
            send_data.imu_data_1 = 0;
            send_data.imu_data_2 = 0;            
            send_data.imu_data_4 = 0;            
            char buffer[sizeof(struct Data)];
            memcpy(buffer, &send_data, sizeof(struct Data));
            std::unique_lock<std::mutex> lock(send_mutex_itx_to_drone);
            send(bt_server_socket, buffer, sizeof(struct Data), 0);
            lock.unlock();            
        }

        long long message_id = rcvd_data.id;
        while (!message_queue.empty()) {
            long long frontMessage = message_queue.front();
            if (frontMessage == message_id) {
                message_queue.pop();  
                break; 
            } else {
                message_queue.pop();  
            }
        }
    }
    else if (device == "drone")
    {
        switch (rcvd_data.info)
        {
        case imu:
        //send ACk with id 
        //we should define a function as below for sending anything
        // send_message(Data message)
        imu_flag = true;            
        break;
        case start:
            //elon function
        break;            
        case land:
            //elon function
        break;          
        default:
            std::cout << "Invalid Information.\n";
            break;
        }         
    }
    else
    {
        std::cout << "Invalid device... exiting.\n";
    }
}

void receive_data(int client) 
{
    char buf[1024];
    int bytes_read{};

    while (true) {
        memset(buf, 0, sizeof(buf));
        bytes_read = read(client, buf, sizeof(buf));
        if (bytes_read > 0)
        {
            struct Data rcvd_data;
            bytes_read = read(client, buf, sizeof(buf));
            memcpy(&rcvd_data, buf, sizeof(struct Data));
            message_handler(rcvd_data);
            // std::cout << "Info: " << informationMap[rcvd_data.info] << "\n";
            // std::cout << "IMU data 1: " << rcvd_data.imu_data_1 << "\n";
            // std::cout << "IMU data 4: " << rcvd_data.imu_data_4 << "\n";
        }
        memset(buf, 0, sizeof(buf));
    }
}

void check_connection
{
    auto start = std::chrono::high_resolution_clock::now();
    while (true)
    {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        if (duration.count() >= EMERGENCY_LANDING_THRESHOLD_MS) 
        {
            std::cout << "Emergency land" << '\n';
            //call Elons emergency land function
        }
        if (imu_flag == true) //reset counter
        {
            imu_flag = false;
            start = std::chrono::high_resolution_clock::now();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int run_bt_server()
{
    int sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    sockaddr_rc loc_addr = { 0 };
    loc_addr.rc_family = AF_BLUETOOTH;
    bdaddr_t bdaddr = {0, 0, 0, 0, 0, 0}; // Initialize a bdaddr_t variable
    loc_addr.rc_bdaddr = bdaddr;
    loc_addr.rc_channel = 1;  // RFCOMM channel to use (e.g., 1).
    
    long long message_id = 0;
    

    bind(sock, (struct sockaddr*)&loc_addr, sizeof(loc_addr));
    listen(sock, 1);  // Allow one connection at a time.

    std::cout << "Waiting for a BT connection...\n";

    sockaddr_rc rem_addr = { 0 };
    socklen_t opt = sizeof(rem_addr);
    int client = accept(sock, (struct sockaddr*)&rem_addr, &opt);
    std::cout << "BT connection accepted\n";

    std::thread receiveThread(receive_data, client);
    bt_server_socket = client;
    while (true) 
    {   
        message_id+=1;
        struct Data send_data = {}; // Send data to the receiver
        send_data.id = message_id;
        send_data.info = ping;
        send_data.imu_data_1 = 1;
        send_data.imu_data_2 = 1;
        //send_data.imu_data_3 = 5.2;
        send_data.imu_data_4 = 1.1;
        // Serialize the struct by copying its memory representation into a buffer
        char buffer[sizeof(struct Data)];
        memcpy(buffer, &send_data, sizeof(struct Data));
        std::unique_lock<std::mutex> lock(send_mutex_itx_to_drone);
        send(client, buffer, sizeof(struct Data), 0);
        lock.unlock();
        message_queue.push(message_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    receiveThread.join();
    close(client);
    close(sock);

    return 0;
}

int run_bt_client(std::string remote_connection) 
{
    int sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    int status;
    sockaddr_rc server_addr = { 0 };
    server_addr.rc_family = AF_BLUETOOTH;
    const char* dest = remote_connection.c_str();

    str2ba(dest, &server_addr.rc_bdaddr); // Replace with the server's address.
    server_addr.rc_channel = 1;  // RFCOMM channel to use (must match the server).
    
    for (int i=0; i<5;i++)
    {
        std::cout << "Connecting... Attempt: " << i+1 << "\n";
        status = connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (status == 0)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    if (status < 0) {
        perror("Connection failed");
        close(sock);
        return 0;
    }

    std::thread receiveThread(receive_data, sock);
    std::thread connectionCheckerThread(check_connection);
    bt_client_socket = sock;
    while (true) 
    {   
        struct Data send_data = {}; // Send data to the receiver
        send_data.info = ping;
        send_data.imu_data_1 = 2;
        send_data.imu_data_2 = 2;
        //send_data.imu_data_3 = 5.2;
        send_data.imu_data_4 = 2.2;
        // Serialize the struct by copying its memory representation into a buffer
        char buffer[sizeof(struct Data)];
        memcpy(buffer, &send_data, sizeof(struct Data));
        std::unique_lock<std::mutex> lock(send_mutex_drone_to_itx);
        send(sock, buffer, sizeof(struct Data), 0);
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    receiveThread.join();
    connectionCheckerThread.join();
    close(sock);

    return 0;
}

int run_eth_server()
{
    int server_socket{socket(AF_INET, SOCK_STREAM, 0)};

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY; // Listen on all available interfaces

    bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
    listen(server_socket, 1);
    std::cout << "Waiting for an Ethernet connection...\n";

    sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    int client = accept(server_socket, (struct sockaddr*)&clientAddr, &clientAddrLen);
    std::cout << "Ethernet connection accepted\n";
    
    std::thread receiveThread(receive_data, client);
    eth_server_socket = client;
    while (true) 
    {
        struct Data send_data{}; // Send data to the receiver
        send_data.info = ping;
        send_data.imu_data_1 = 1;
        send_data.imu_data_2 = 1;
        //send_data.imu_data_3 = 5.2;
        send_data.imu_data_4 = 1.1;
        // Serialize the struct by copying its memory representation into a buffer
        char buffer[sizeof(struct Data)];
        memcpy(buffer, &send_data, sizeof(struct Data));
        std::unique_lock<std::mutex> lock(send_mutex_itx_to_px2);
        send(client, buffer, sizeof(struct Data), 0);
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    receiveThread.join();
    close(client);
    close(server_socket);

    return 0;
}

int run_eth_client(std::string remote_connection)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    int status;

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = inet_addr(remote_connection.c_str());

    for (int i=0; i<5;i++)
    {
        std::cout << "Connecting... Attempt: " << i+1 << "\n";
        status = connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (status == 0)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    if (status < 0) {
        perror("Connection failed");
        close(sock);
        return 0;
    }
    eth_client_socket = sock;
    std::thread receiveThread(receive_data, sock);    
                
    while (true) 
    {
        struct Data send_data = {}; // Send data to the receiver
        send_data.info = ping;
        send_data.imu_data_1 = 2;
        send_data.imu_data_2 = 2;
        //send_data.imu_data_3 = 5.2;
        send_data.imu_data_4 = 2.2;
        // Serialize the struct by copying its memory representation into a buffer
        char buffer[sizeof(struct Data)];
        memcpy(buffer, &send_data, sizeof(struct Data));
        std::unique_lock<std::mutex> lock(send_mutex_px2_to_itx);
        send(sock, buffer, sizeof(struct Data), 0);
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    receiveThread.join();
    close(sock);

    return 0;
}

std::map<std::string, std::string> parse_ini_file(const std::string &filename)
{
    std::map<std::string, std::string> config;

    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error: Could not open " << filename << std::endl;
        return config;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // Split the line into key and value
        size_t delimiterPos = line.find('=');
        if (delimiterPos != std::string::npos)
        {
            std::string key = line.substr(0, delimiterPos);
            std::string value = line.substr(delimiterPos + 1);
            config[key] = value;
        }
    }

    return config;
} 	