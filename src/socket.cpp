#include "socket.h"
#include <iostream>

SocketServer::SocketServer(int port) {   
    int opt_socket = 1;
    int addrlen = sizeof(address);
    double TX_torques[2] = {10, -10};
    double RX_state[4] = {0, 0, 0, 0};

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                   &opt_socket, sizeof(opt_socket))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *) &address,
             sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *) &address,
                              (socklen_t *) &addrlen)) < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }
}

void SocketServer::readSocket(void* buffer, int len) {
    read(new_socket, buffer, len);
}

void SocketServer::writeSocket(const void* buffer, int len) {
    send(new_socket, buffer, len, 0);
}

void SocketServer::closeSocket() {
    close(new_socket);
    shutdown(server_fd, SHUT_RDWR);
}

SocketClient::SocketClient(int port) {  
    if ((new_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        exit(EXIT_FAILURE);
    }
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
        exit(EXIT_FAILURE);
    }
    if (connect(new_socket, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        printf("\nConnection Failed \n");
        exit(EXIT_FAILURE);
    }
}

void SocketClient::readSocket(void* buffer, int len) {
    read(new_socket, buffer, len);
}

void SocketClient::writeSocket(const void* buffer, int len) {
    send(new_socket, buffer, len, 0);
}

void SocketClient::closeSocket() {
    close(new_socket);
}