#pragma once

#include "Segway.h"
#include "TCPServer.h"

#include <esp_log.h>

class Telemetry : public TCPServer {
  public:
    Telemetry(const int port, const Segway *robot);

  private:
    const Segway *robot = nullptr;
    static void service(const Telemetry *p, const int fd);
};

Telemetry::Telemetry(const int port, const Segway *r) : TCPServer(port, (TCPServer::service_t)service) {
    robot = r;
}

void Telemetry::service(const Telemetry *p, const int fd) {
    while (true) {
        int x = dprintf(fd, "%i | %.1f %.1f | %.1f | %.1f %.1f %.1f | %i %i\n",
                        xTaskGetTickCount(),
                        p->robot->Gyro_x, p->robot->Angle_x, p->robot->Angle,
                        p->robot->tiltPIDOutput, p->robot->speedPIDOutput, p->robot->turnPIDOutput,
                        p->robot->leftMotorPWM, p->robot->rightMotorPWM);
        if (x < 0) return;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
