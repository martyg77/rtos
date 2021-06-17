#include "TCPServer.h"

#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/err.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <nvs_flash.h>
#include <string.h>
#include <sys/param.h>
#include <assert.h>

void tcp_server_task(TCPServer *p) {

    // Create unbound socket
    p->listener_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    assert(p->listener_fd >= 0);

    // Assign local address to unnamed socket
    bzero(&p->dest, sizeof(p->dest));
    p->dest.sin_family = AF_INET;
    p->dest.sin_port = htons(p->port);
    p->dest.sin_addr.s_addr = htonl(INADDR_ANY);

    int rc = bind(p->listener_fd, (sockaddr *)&p->dest, sizeof(p->dest));
    assert(rc == 0);

    // Mark the socket as accepting connections
    rc = listen(p->listener_fd, 1);
    assert(rc == 0);

    while (true) {

        // Accept a new connection
        ESP_LOGI(p->label, "Listening");
        socklen_t len = sizeof(p->src);
        p->accepted_fd = accept(p->listener_fd, (sockaddr *)&p->src, &len);
        assert(p->accepted_fd >= 0);

        // Log the incoming client
        char addr_str[32];
        inet_ntoa_r(p->src.sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(p->label, "Accepted %s:%i", addr_str, p->src.sin_port);

        // Redirect stdin/stdout to client connection
        // Note stdin/stdout are process-specific, can be direclty assigned
        FILE *in = stdin;
        FILE *out = stdout;
        stdin = fdopen(p->accepted_fd, "r");
        stdout = fdopen(p->accepted_fd, "w");

        // Process the incoming stream until user quits or something goes wrong
        p->service(p, p->accepted_fd);

        // Close the connection and recycle for the next client
        fclose(stdin);
        fclose(stdout);
        stdin = in;
        stdout = out;
        shutdown(p->accepted_fd, 0);
        close(p->accepted_fd);
    }

    // We should never reach this point
    close(p->listener_fd);
    vTaskDelete(nullptr);
}

TCPServer::TCPServer(const int p, const service_t s) {
    port = p;
    service = s;
    snprintf(label, sizeof(label), "tcp:%i", port);
    xTaskCreate((TaskFunction_t)tcp_server_task, label, 4096, this, 5, nullptr);
}

void Echo::stdio(const TCPServer *p, const int fd) {
    char s[256];

    while (fgets(s, sizeof(s), stdin)) {
        puts(s);
        fflush(stdout);
    }
}

void Echo::socket(const TCPServer *p, const int fd) {
    int len;
    char rx_buffer[128];

    static const char *TAGE = "echo";

    do {
        len = recv(fd, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAGE, "Error occurred during receiving: errno %d - %s", errno, strerror(errno));
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
                    ESP_LOGE(TAGE, "Error occurred during sending: errno %d - %s", errno, strerror(errno));
                }
                to_write -= written;
            }
        }
    } while (len > 0);
}
