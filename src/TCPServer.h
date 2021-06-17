// Unix TCP socket server

// Ref. https://pubs.opengroup.org/onlinepubs/007908799/xsh/stdio.html

#pragma once

#include <esp_log.h>
#include <lwip/sockets.h>
#include <sys/errno.h>

// TODO some of these members should be private

class TCPServer {
  public:
    // Socket server prototype
    // Called with stdin/stdout redirected to socket client
    typedef void (*service_t)(const TCPServer *p, const int fd);

    TCPServer(const int port, const service_t service);

    int port = 0;
    service_t service = nullptr;
    char label[32] = "";

    int listener_fd = 0;
    int accepted_fd = 0;

    sockaddr_in dest;
    sockaddr_in src;
};

// Echo service, useful for troubleshooting

class Echo : public TCPServer {
  public:
    Echo(const int port) : TCPServer(port, stdio) {}

  private:
    static void stdio(const TCPServer *p, const int fd);
    static void socket(const TCPServer *p, const int fd);
};
