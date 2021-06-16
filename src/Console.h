#pragma once

#include "TCPServer.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Console : public TCPServer {
  public:
    Console(const int port);

  private:
    int port = 0;
    static void service(const Console *p, const int fd);
};
