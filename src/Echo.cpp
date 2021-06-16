#include "Echo.h"

#pragma GCC optimize ("-O0")

void Echo::stdio(const TCPServer *p, const int fd) {
    // stdin/stdout are process specific
    stdin = fdopen(fd, "r");
    stdout = fdopen(fd, "w");

    char s[256];

    while (fgets(s, sizeof(s), stdin)) {
        puts(s);
        fflush(stdout);
    }

    fclose(stdin);
    fclose(stdout);
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
