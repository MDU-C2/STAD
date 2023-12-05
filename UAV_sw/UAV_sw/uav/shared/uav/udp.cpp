#include "udp.h"

#include <cstring> // for memset
#include <iostream>
#include <thread>
#include <arpa/inet.h>
#include <unistd.h>
#include <mutex>

#define BUFFER_SIZE 1024

// This mutex allows multiple threads to use the udp_send_msg
// without any external synchronization
static std::mutex g_send_mutex;

//static int g_port = 12345;
static int g_sock = -1;

static bool g_thread_join;
std::thread g_thread_recv;
static void(*g_recv_msg_callback)(void*, size_t) = nullptr;

struct sockaddr_in g_addr_other;

static void udp_recv_task()
{
    char buffer[BUFFER_SIZE];
    while (!g_thread_join) {
        struct sockaddr_in clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);

        ssize_t bytes_read = recvfrom(g_sock, buffer, sizeof(buffer), 0,
                                        (struct sockaddr*)&clientAddr, &clientAddrLen);

        if (bytes_read > 0) {
            // std::cout << "Received: " << (const char*)buffer << " from " << inet_ntoa(clientAddr.sin_addr) << "\n";
            if (!g_recv_msg_callback)
                std::cout << "Received msg, but no callback is set!\n";
            else
                g_recv_msg_callback(buffer, bytes_read);
        }
    }
}

void udp_send_msg(const void* buffer, size_t size)
{
    std::unique_lock<std::mutex> lock(g_send_mutex);
    
    int bytes_sent = sendto(g_sock, buffer, size, 0,
            (struct sockaddr*)&g_addr_other, sizeof(g_addr_other));

    if (bytes_sent <= 0)
        std::cout << "UDP: Failed to send msg.\n";
}

bool udp_create(UDP_Type type, void(*recv_callback)(void*, size_t))
{
    std::cout << "UDP: Creating ";
    switch (type) {
    case UDP_Type::Server:
        std::cout << "server...\n";
        break;
    case UDP_Type::Client:
        std::cout << "client...\n";
        break;
    }

    g_recv_msg_callback = recv_callback;

    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock == -1) {
        std::cerr << "UDP: socket() failed.\n";
        return false;
    }

    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(g_addr_other);

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = 8888; // Let OS decide port

    char* str = 0;

    switch (type) {
        case UDP_Type::Server: {
            str = "Server";
            
            // We are the server so we must bind the socket
            int bound = bind(g_sock, (struct sockaddr*)&addr, sizeof(addr));
            if (bound == -1) {
                std::cerr << "UDP: bind() failed.\n";
                close(g_sock);
                return false;
            }

            // Listen for a client to "connect"
            char buffer[BUFFER_SIZE];
            std::cout << "Waiting for client to \"connect\".\n";
            ssize_t bytes_read = recvfrom(g_sock, buffer, sizeof(buffer), 0,
                                (struct sockaddr*)&g_addr_other, &addr_len);

            std::cout << "Success receiving from client.\n";

            const char* msg = "Response";
            udp_send_msg(msg, strlen(msg) + 1);

            break;
        }
        case UDP_Type::Client: {
            str = "Client";
            addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Let OS decide port
            memcpy(&g_addr_other, &addr, sizeof(addr));
            udp_send_msg(str, strlen(str) + 1);

            char buffer[BUFFER_SIZE];
            std::cout << "Waiting for server response\n";
            ssize_t bytes_read = recvfrom(g_sock, buffer, sizeof(buffer), 0,
                (struct sockaddr*)&g_addr_other, &addr_len);

            std::cout << "Server responded with: " << buffer << "\n";

            break;
        }
    }

    std::cout << "UDP: " << str << " created!\n";

    g_thread_recv = std::thread(udp_recv_task);

    return true;
}

void udp_destroy()
{
    g_thread_join = true;
    close(g_sock);
}