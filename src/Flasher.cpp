#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "Flasher.h"

void flasher(void *p)
{
    Flasher::flasherConfig *c = (Flasher::flasherConfig *) p;

    gpio_pad_select_gpio(c->pin); // PinMux magic
    gpio_set_direction(c->pin, GPIO_MODE_OUTPUT);
    gpio_pullup_dis(c->pin);
    gpio_pulldown_dis(c->pin);
    gpio_intr_disable(c->pin);

    while (true)
    {
        gpio_set_level(c->pin, 1);
        vTaskDelay(c->speed / portTICK_PERIOD_MS);
        gpio_set_level(c->pin, 0);
        vTaskDelay(c->speed * 3 / portTICK_PERIOD_MS);
    }
}

Flasher::Flasher(flasherConfig c)
{
    this->config = c;
    xTaskCreate(flasher, "flasher", configMINIMAL_STACK_SIZE, (void *) &this->config, 0, &this->handle);
}

Flasher::~Flasher() {
    vTaskDelete(this->handle);
}