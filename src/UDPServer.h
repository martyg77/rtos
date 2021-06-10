// TODO (mistly) untested code

#pragma once

#include <lwip/sockets.h>

class UDPServer {
  public:
    typedef void (*service_t)(const UDPServer *p, const int fd);

    UDPServer(const int port, const service_t service);

    int port = 0;

    int fd = 0;

    sockaddr_in dest;
    sockaddr_in src;

    service_t service = nullptr;
};
