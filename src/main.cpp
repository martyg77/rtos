#include "Flasher.h"

const gpio_num_t GPIO_LED_RED = GPIO_NUM_0;
const gpio_num_t GPIO_LED_GREEN = GPIO_NUM_2;
const gpio_num_t GPIO_LED_BLUE = GPIO_NUM_4;

extern "C" { void app_main(); }

Flasher red({GPIO_LED_RED, 250});
Flasher green({GPIO_LED_GREEN, 300});
Flasher blue({GPIO_LED_BLUE, 500});

void app_main() {}
