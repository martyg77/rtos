#include <driver/gpio.h>
#include "Flasher.h"
#include "Button.hpp"
#include "Encoder.h"

const gpio_num_t GPIO_LED_RED = GPIO_NUM_0;
const gpio_num_t GPIO_LED_GREEN = GPIO_NUM_2;
const gpio_num_t GPIO_LED_BLUE = GPIO_NUM_4;

Flasher red(GPIO_LED_RED, 250);
Flasher green(GPIO_LED_GREEN, 300);
Flasher blue(GPIO_LED_BLUE, 500);

Button ky040_button(GPIO_NUM_25);
Encoder ky040_encoder(GPIO_NUM_26, GPIO_NUM_27);
Button nil_button(GPIO_NUM_39);

extern "C" { void app_main(); }

void app_main()
{
    // This procedure can only be called once
    gpio_install_isr_service(0);
}
