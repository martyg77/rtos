#pragma once

#include "TCPServer.h"

#include <esp_log.h>
#include <sys/errno.h>

class Echo : public TCPServer {
  public:
    Echo(const int port) : TCPServer(port, iostream) {}

  private:
    static void iostream(const TCPServer *p, const int fd);
    static void iotest(const TCPServer *p, const int fd);
    static void stdio(const TCPServer *p, const int fd);
};

// Ref. https://pubs.opengroup.org/onlinepubs/007908799/xsh/stdio.html

// TODO need to set up stream as unbuffered, line buffered makes most sense for Echo ref. setbuf() setvbuf()
// TODO byte-oriented vs wide-oriented

#pragma GCC optimize ("-O0")

void Echo::iostream(const TCPServer *p, const int fd) {
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

void Echo::iotest(const TCPServer *p, const int fd) {
    FILE *ostdin = stdin;
    FILE *ostdout = stdout;
    fprintf(ostdout, "%i %p %p\n", fd, stdin, stdout);

    FILE *nstdin = fdopen(fd, "r");
    if (!nstdin) fprintf(ostdout, "nstdin error - %s", strerror(errno));
    FILE *nstdout = fdopen(fd, "w");
    if (!nstdout) fprintf(ostdout, "nstdout error - %s", strerror(errno));

    stdin = nstdin;
    stdout = nstdout;
    fprintf(ostdout, "%i %p %p\n", fd, stdin, stdout);
    fprintf(stdout, "%i %p %p\n", fd, stdin, stdout);
    fflush(stdout);

    fclose(ostdin);
    fclose(ostdout);
    fclose(nstdin);
    fclose(nstdout);
}

void Echo::stdio(const TCPServer *p, const int fd) {
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
