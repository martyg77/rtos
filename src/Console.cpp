#include "Console.h"

#include <argtable3/argtable3.h>
#include <driver/uart.h>
#include <esp_console.h>
#include <esp_log.h>
#include <linenoise/linenoise.h>
#include <string.h>

// TOOD re-integrate linenoise library, clean up main loop

void Console::service(const Console *p, const int fd) {

    stdin = fdopen(p->accepted_fd, "r");
    stdout = fdopen(p->accepted_fd, "w");

    char s[256];
    int rc;

    while (fgets(s, sizeof(s), stdin)) {
        strtok(s, "\n"); // Strip trailing newline
        strtok(s, "\r"); // Strip trailing carriage return
        esp_err_t err = esp_console_run(s, &rc);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && rc != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", rc, esp_err_to_name(rc));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        fflush(stdout);
    }
    fclose(stdin);
    fclose(stdout);
}

static float Kp, Ki, Kd;

static struct {
    struct arg_dbl *Kp = arg_dbl1(nullptr, nullptr, "<Kp>", "PID proportional gain");
    struct arg_dbl *Ki = arg_dbl1(nullptr, nullptr, "<Ki>", "PID integral gain");
    struct arg_dbl *Kd = arg_dbl1(nullptr, nullptr, "<Kd>", "PID differential gain");
    struct arg_end *end = arg_end(5);
} tilt_args;

int tilt(int argc, char **argv) {
    if (arg_parse(argc, argv, (void **)&tilt_args) > 0) {
        arg_print_errors(stdout, tilt_args.end, argv[0]);
        return 1;
    }

    Kp = tilt_args.Kp->dval[0];
    Ki = tilt_args.Ki->dval[0];
    Kd = tilt_args.Kd->dval[0];

    return 0;
}

int pid(int argc, char **argv) {
    printf("Tilt: Kp %f Ki %f Kd %f\n", Kp, Ki, Kd);
    return 0;
}

Console::Console(const int p) : TCPServer(p, (TCPServer::service_t)service) {
    port = p;

    esp_console_config_t console_config = ESP_CONSOLE_CONFIG_DEFAULT();
    esp_console_init(&console_config);

    esp_console_register_help_command();

    const esp_console_cmd_t tilt_cmd = {
        .command = "tilt",
        .help = "Adjust tilt PID gains",
        .hint = nullptr,
        .func = tilt,
        .argtable = (void *)&tilt_args,
    };
    esp_console_cmd_register(&tilt_cmd);

    const esp_console_cmd_t pid_cmd = {
        .command = "pid",
        .help = "Display adjustable PID gains",
        .hint = nullptr,
        .func = pid,
        .argtable = arg_end(5),
    };
    esp_console_cmd_register(&pid_cmd);
}
