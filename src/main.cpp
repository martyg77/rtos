#include <driver/gpio.h>

#include "Button.h"
#include "Encoder.h"
#include "Flasher.h"
#include "MyStepper.h"
#include "GUI.h"

const gpio_num_t GPIO_LED_RED = GPIO_NUM_0;
const gpio_num_t GPIO_LED_GREEN = GPIO_NUM_2;
const gpio_num_t GPIO_LED_BLUE = GPIO_NUM_4;

Flasher red(GPIO_LED_RED, 250);
Flasher green(GPIO_LED_GREEN, 300);
Flasher blue(GPIO_LED_BLUE, 500);

// MyStepper stepper(GPIO_NUM_5, GPIO_NUM_19, GPIO_NUM_18);

extern "C" { void app_main(); }

void app_main() {
    // This procedure can only be called once
    gpio_install_isr_service(0);  // TODO Needs to run *BEFORE* constuctors!
    
    xTaskCreatePinnedToCore(guiProcess, "gui", configMINIMAL_STACK_SIZE * 8, NULL, 0, NULL, guiCpuCore); 
}
