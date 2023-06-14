#include <iostream>
#include "socket.h"
#include <unistd.h>

int main() {
    std::cout << "Running Controller executable" << std::endl;
    SocketServer socket(SocketServer::DEFAULT_PORT);

    double RX_state[4] = {0, 0, 0, 0};
    double TX_torque[2] = {10, -10};

    for (int ii = 0; ii < 10; ii++) {
        socket.readSocket(&RX_state, sizeof(RX_state));
        socket.writeSocket(&TX_torque, sizeof(TX_torque));

        std::cout << "Received State: \n\t[" << RX_state[0] << ", " << RX_state[1] << ", " << RX_state[2] << ", " << RX_state[3] << "]" << std::endl; 
        sleep(0.01);
    }
    socket.closeSocket();
}
