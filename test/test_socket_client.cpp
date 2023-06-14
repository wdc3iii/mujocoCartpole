#include <iostream>
#include "socket.h"
#include <unistd.h>

int main() {
    std::cout << "Running Socket Client executable" << std::endl;
    SocketClient socket(SocketClient::DEFAULT_PORT);

    double TX_state[4] = {0, 1, 2, 3};
    double RX_torque[2] = {0, 0};

    for (int ii = 0; ii < 10; ii++) {
        socket.writeSocket(&TX_state, sizeof(TX_state));
        socket.readSocket(&RX_torque, sizeof(RX_torque));
        std::cout << "Received Torques: \n\t[" << RX_torque[0] << ", " << RX_torque[1] << "]" << std::endl; 
        sleep(0.01);
    }
    socket.closeSocket();
}
