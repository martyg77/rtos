// TODO (mistly) untested code

#include "UDPServer.h"

#include <assert.h>
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

static const char *TAG = "UDP";

void udp_server_task(UDPServer *p) {

    // Create unbound socket
    p->fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    assert(p->fd >= 0);
    ESP_LOGI(TAG, "Socket created");

    // Assign local address to unnamed socket
    bzero(&p->dest, sizeof(p->dest));
    p->dest.sin_family = AF_INET;
    p->dest.sin_port = htons(p->port);
    p->dest.sin_addr.s_addr = htonl(INADDR_ANY);

    int rc = bind(p->fd, (sockaddr *)&p->dest, sizeof(p->dest));
    assert(rc == 0);
    ESP_LOGI(TAG, "Socket bound, port %d", p->port);

    while (true) {

        // Peek into opening packet from peer, extract source address
        ESP_LOGI(TAG, "Socket listening");
        char buffer[256];
        socklen_t srclen = sizeof(p->src);
        int msglen = recvfrom(p->fd, buffer, sizeof(buffer) - 1, MSG_PEEK, (sockaddr *)&p->src, &srclen);

        if (msglen > 0) {

            // Human-readable (dotted-quad) source address
            char addr_str[32];
            inet_ntoa_r(p->src.sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
            ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

            // Process the incoming stream
            p->service(p, p->fd);
        }

        // Close the connection and recycle for the next client
        shutdown(p->fd, 0);
        close(p->fd);
    }

    // We should never reach this point
    close(p->fd);
    vTaskDelete(nullptr);
}

UDPServer::UDPServer(const int p, const service_t s) {
    port = p;
    service = s;
    // TODO include port number in task name string
    xTaskCreate((TaskFunction_t)udp_server_task, "tcp_server", 4096, this, 5, nullptr);
}
