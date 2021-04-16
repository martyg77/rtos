#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "Flasher.h"

const gpio_num_t GPIO_LED_RED = GPIO_NUM_0;
const gpio_num_t GPIO_LED_GREEN = GPIO_NUM_2;
const gpio_num_t GPIO_LED_BLUE = GPIO_NUM_4;

void init_led(gpio_num_t led) {
    gpio_pad_select_gpio(led);
    gpio_set_direction(led, GPIO_MODE_OUTPUT);
    gpio_set_level(led, 0);
}

void flasher(void *p)
{
    const gpio_num_t led = GPIO_LED_GREEN;
    Flasher f(led, 300);
}

void recurse(void *p)
{
    while (1)
        (vTaskDelay(1000 / portTICK_PERIOD_MS));
    recurse(NULL);
}

extern "C" { void app_main(); }

void app_main()
{
    printf("Turning off RGB LEDs\n");
    init_led(GPIO_LED_RED);
    init_led(GPIO_LED_GREEN);
    init_led(GPIO_LED_BLUE);

    printf("Spawning flasher task\n");
    xTaskCreate(flasher, "flasher", configMINIMAL_STACK_SIZE, NULL, 0, NULL);

    printf("Spawning stack overflow\n");
    xTaskCreate(recurse, "recurse", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
}
