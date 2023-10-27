#include "comms.h"


int main() 
{
    std::map<std::string, std::string> config_data = parse_ini_file(CONFIG);
    std::string device = config_data["device"];
    std::string remote_connection = config_data["remote_connection"];

    if (device == "itx")
    {
        run_bt_server();
    }
    else if (device == "drone")
    {        
        run_bt_client(remote_connection);
    }
    else
    {
        std::cout << "Invalid device... exiting.\n";
    }
    
    return 0;
}