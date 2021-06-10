#pragma once

#include "TCPServer.h"

#include <esp_log.h>

class Echo : public TCPServer {
  public:
    Echo(const int port) : TCPServer(port, service) {}

  private:
    static void service(const TCPServer *p, const int fd);
};

void Echo::service(const TCPServer *p, const int fd) {
    int len;
    char rx_buffer[128];

    static const char *TAGE = "echo";

    do {
        len = recv(fd, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAGE, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAGE, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAGE, "Received %d bytes: %s", len, rx_buffer);

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.
            int to_write = len;
            while (to_write > 0) {
                int written = send(fd, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAGE, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
        }
    } while (len > 0);
}
