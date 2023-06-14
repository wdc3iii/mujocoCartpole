#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

#define PORT 8080
#ifndef SOCKET_H
#define SOCKET_H

class SocketServer {
    private:
        int server_fd;
        int new_socket;
        struct sockaddr_in address;

    public:
        const static int DEFAULT_PORT = PORT;
        SocketServer(int port);
        void readSocket(void* buffer, int len);
        void writeSocket(const void* buffer, int len);
        void closeSocket();
};

class SocketClient {
    private:
        int new_socket;
        struct sockaddr_in serv_addr;

    public:
        const static int DEFAULT_PORT = PORT;
        SocketClient(int port);
        void readSocket(void* buffer, int len);
        void writeSocket(const void* buffer, int len);
        void closeSocket();
};

#endif
