#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "Flasher.h"

void flasher(void *p)
{
    Flasher::flasherConfig *c = (Flasher::flasherConfig *) p;

    gpio_pad_select_gpio(c->led);
    gpio_set_direction(c->led, GPIO_MODE_OUTPUT);
    gpio_set_level(c->led, 0);

    while (true)
    {
        gpio_set_level(c->led, 1);
        vTaskDelay(c->speed / portTICK_PERIOD_MS);
        gpio_set_level(c->led, 0);
        vTaskDelay(c->speed * 3 / portTICK_PERIOD_MS);
    }
}

Flasher::Flasher(flasherConfig config) {
    this->config = config;
    xTaskCreate(flasher, "flasher", configMINIMAL_STACK_SIZE, (void *) &this->config, 0, &this->handle);
}

Flasher::~Flasher() {
    vTaskDelete(this->handle);
}