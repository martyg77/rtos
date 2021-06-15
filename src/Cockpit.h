// stty -icanon && nc <host> 5555

// a.k.a. Helm

#pragma once

#include "TCPServer.h"
#include "Segway.h"

#include <esp_log.h>

class Cockpit : public TCPServer {
  public:
    Cockpit(const int port, Segway *robot);

  private:
    Segway *robot = nullptr;
    static void service(const Cockpit *p, const int fd);
};

Cockpit::Cockpit(const int port, Segway *r) : TCPServer(port, (TCPServer::service_t)service) {
    robot = r;
}

void Cockpit::service(const Cockpit *p, const int fd) {
    char c;

    static const char *TAG = "cockpit";

    // Design intent is to use numeric keypad as makeshift joystick
    while (recv(fd, &c, sizeof(c), 0) > 0) {
        ESP_LOGW(TAG, "%c", c);
        switch (c) {
        case '2':
            p->robot->speedSetPoint -= 25;
            break;
        case '4':
            p->robot->turnSetPoint = (p->robot->turnSetPoint - 30) % 360;
            break;
        case '6':
            p->robot->turnSetPoint = (p->robot->turnSetPoint + 30) % 360;
            break;
        case '8':
            p->robot->speedSetPoint += 25;
            break;
        default:
            p->robot->stop();
            break;
        }
    }
}
