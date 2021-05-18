#include "Flasher.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

typedef struct {
    gpio_num_t pin;
    int speed;
} flasherProcess_t;

void flasherProcess(flasherProcess_t *tcb) {
    gpio_pad_select_gpio(tcb->pin);
    gpio_config_t g = {
        .pin_bit_mask = 1ull << tcb->pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&g);

    while (true) {
        gpio_set_level(tcb->pin, 1);
        vTaskDelay(tcb->speed / portTICK_PERIOD_MS);
        gpio_set_level(tcb->pin, 0);
        vTaskDelay(tcb->speed * 3 / portTICK_PERIOD_MS);
    }
}

Flasher::Flasher(const gpio_num_t p, const uint s) {
    pin = p;
    speed = s;
    tcb = new (flasherProcess_t){pin, speed};
    xTaskCreate((TaskFunction_t)flasherProcess, "flasher", configMINIMAL_STACK_SIZE * 4, tcb, 0, &handle);
}

Flasher::~Flasher() {
    delete ((flasherProcess_t *)tcb);
    vTaskDelete(handle);
}
