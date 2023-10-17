#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <string.h>

struct Data {
    int x;
    int y;
    char info[10];
};

int main(int argc, char **argv)
{
    printf("Client mode enabled\n");
    struct sockaddr_rc addr = { 0 };
    int s, status;
    char dest[18] = "D8:3A:DD:53:F1:3D";
    struct Data new_data;
    new_data.x = 5;
    new_data.y = 3;
    strncpy(new_data.info, "Hejsan", sizeof(new_data.info));
    // Serialize the struct by copying its memory representation into a buffer
    char buffer[sizeof(struct Data)];
    memcpy(buffer, &new_data, sizeof(struct Data));

    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );

    // connect to server
    status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

    // send a message
    if( status == 0 ) {
	while (1) {
        	status = write(s, buffer, sizeof(struct Data));
    		sleep(3);
	}
    }
    if( status < 0 ) perror("uh oh");

    close(s);
    return 0;
}
