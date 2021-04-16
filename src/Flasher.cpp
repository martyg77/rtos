#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "Flasher.h"

Flasher::Flasher(gpio_num_t led, int speed)
{
    this->led = led;
    this->speed = speed;

    gpio_pad_select_gpio(led);
    gpio_set_direction(led, GPIO_MODE_OUTPUT);
    gpio_set_level(led, 0);

    while (1)
    {
        gpio_set_level(led, 1);
        vTaskDelay(speed / portTICK_PERIOD_MS);
        gpio_set_level(led, 0);
        vTaskDelay(speed * 3 / portTICK_PERIOD_MS);
    }
}