#include "comms.h"


void printQueue(const std::queue<long long>& q) {
    std::queue<long long> tempQueue = q; // Make a temporary copy
    while (!tempQueue.empty()) {
        long long element = tempQueue.front();
        std::cout << element << " ";
        tempQueue.pop();
    }
}

void print_queue_size(){
    std::cout << "This function print the size of the message queue each HELLO_RATE_MS ms...\n";
    while (true)
    {       
        std::cout << "Size of the message queue: " << message_queue.size() << "\n";
        // for (size_t i = 0; i < message_queue.size(); i++)
        // {
        //     std::cout << message_queue[i]<< ", " ; 
        // }

        printQueue(message_queue);
        std::cout << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(HELLO_RATE_MS));
    }
}

int main() 
{
    imu_flag = false;  
    config_data = parse_ini_file(CONFIG);
    device = config_data["device"];     

    std::thread run_bt_server_Thread(run_bt_server);
    std::thread print_queue_size_Thread(print_queue_size);

    print_queue_size_Thread.join();
    run_bt_server_Thread.join();
}

//count acks and sents 
//are all sent, acked ?
// are they acked in order // find missed acks
// drop connection 1- server 2-client
// send 5 messages in order and see if it will be received ack for theme in order.