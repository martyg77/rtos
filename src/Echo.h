// TCP Echo service, useful for troubleshooting

// Ref. https://pubs.opengroup.org/onlinepubs/007908799/xsh/stdio.html

// TODO need to set up stream as unbuffered, line buffered makes most sense for Echo ref. setbuf() setvbuf()
// TODO byte-oriented vs wide-oriented

#pragma once

#include "TCPServer.h"

#include <esp_log.h>
#include <sys/errno.h>

class Echo : public TCPServer {
  public:
    Echo(const int port) : TCPServer(port, stdio) {}

  private:
    static void stdio(const TCPServer *p, const int fd);
    static void socket(const TCPServer *p, const int fd);
};

