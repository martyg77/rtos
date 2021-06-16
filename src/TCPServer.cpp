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

        char addr_str[32];
        inet_ntoa_r(p->src.sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(p->label, "Accepted %s:%i", addr_str, p->src.sin_port);

        // Process the incoming stream until user quits or something goes wrong
        p->service(p, p->accepted_fd);

        // Close the connection and recycle for the next client
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
