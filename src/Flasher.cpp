#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "Flasher.h"

typedef struct {
    gpio_num_t pin;
    int speed;
} flasherProcess_t;

void flasherProcess(void *p) {
    flasherProcess_t *tcb = (flasherProcess_t *) p;

    gpio_pad_select_gpio(tcb->pin); // PinMux magic
    gpio_config_t g = {
        .pin_bit_mask = 1ull << tcb->pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&g);

    while (true)
    {
        gpio_set_level(tcb->pin, 1);
        vTaskDelay(tcb->speed / portTICK_PERIOD_MS);
        gpio_set_level(tcb->pin, 0);
        vTaskDelay(tcb->speed * 3 / portTICK_PERIOD_MS);
    }
}

Flasher::Flasher(gpio_num_t p, int s)
{
    pin = p;
    speed = s;
    tcb = new(flasherProcess_t) { pin, speed };
    xTaskCreate(flasherProcess, "flasher", configMINIMAL_STACK_SIZE * 4, tcb, 0, &handle);
}

Flasher::~Flasher() {
    delete((flasherProcess_t*) tcb);
    vTaskDelete(handle);
}
