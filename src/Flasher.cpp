#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "Flasher.h"

void flasher(void *p)
{
    Flasher::flasherConfig *c = (Flasher::flasherConfig *) p;

    while (true)
    {
        gpio_set_level(c->led, 1);
        vTaskDelay(c->speed / portTICK_PERIOD_MS);
        gpio_set_level(c->led, 0);
        vTaskDelay(c->speed * 3 / portTICK_PERIOD_MS);
    }
}

Flasher::Flasher(flasherConfig c)
{
    gpio_pad_select_gpio(c.led); // PinMux magic
    gpio_set_direction(c.led, GPIO_MODE_OUTPUT);
    gpio_pullup_dis(c.led);
    gpio_pulldown_dis(c.led);
    gpio_intr_disable(c.led);

    this->config = c;
    xTaskCreate(flasher, "flasher", configMINIMAL_STACK_SIZE, (void *)&this->config, 0, &this->handle);
}

Flasher::~Flasher() {
    vTaskDelete(this->handle);
}