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
    typedef void (*service_t)();
    static void echo();

    TCPServer(const int port, const service_t service);

    int port = 0;
    service_t service = echo;
    char label[32] = "";
    TaskHandle_t task = nullptr;

    int listener_fd = 0;
    int accepted_fd = 0;

    sockaddr_in dest;
    sockaddr_in src;
};
