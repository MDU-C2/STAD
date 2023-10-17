#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

struct Data {
    int x;
    int y;
    char info[10];
};

int main(int argc, char **argv)
{
    struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
    char buf[1024] = { 0 };
    int s, client, bytes_read;
    socklen_t opt = sizeof(rem_addr);

    // allocate socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // bind socket to port 1 of the first available 
    // local bluetooth adapter
    loc_addr.rc_family = AF_BLUETOOTH;
    loc_addr.rc_bdaddr = *BDADDR_ANY;
    loc_addr.rc_channel = (uint8_t) 1;
    bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));

    // put socket into listening mode
    listen(s, 1);

    // accept one connection
    client = accept(s, (struct sockaddr *)&rem_addr, &opt);

    ba2str( &rem_addr.rc_bdaddr, buf );
    fprintf(stderr, "accepted connection from %s\n", buf);
    memset(buf, 0, sizeof(buf));

    // read data from the client
    while(1) {
    	bytes_read = read(client, buf, sizeof(buf));
    	if( bytes_read > 0 ) {
 		struct Data rcvd_data;
		memcpy(&rcvd_data, buf, sizeof(struct Data));
        	printf("x: %d\n", rcvd_data.x);
		printf("y: %d\n", rcvd_data.y);
        	printf("info: %s\n", rcvd_data.info);
    	}
	memset(buf, 0, sizeof(buf));
    }
    // close connection
    close(client);
    close(s);
    return 0;
}
