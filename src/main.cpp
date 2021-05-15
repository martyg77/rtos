#include <driver/gpio.h>

#include "Button.h"
#include "Encoder.h"
#include "Flasher.h"
#include "MyStepper.h"
#include "GUI.h"
#include "lvgl.h"

const gpio_num_t GPIO_LED_RED = GPIO_NUM_0;
const gpio_num_t GPIO_LED_GREEN = GPIO_NUM_2;
const gpio_num_t GPIO_LED_BLUE = GPIO_NUM_4;
    
Flasher red(GPIO_LED_RED, 250);
Flasher green(GPIO_LED_GREEN, 300);
Flasher blue(GPIO_LED_BLUE, 500);

extern "C" { void app_main(); }

void app_main() {

    // These procedures can only be called once
    // TODO Needs to run *BEFORE* constuctors!
    gpio_install_isr_service(0);  
    lv_init();

    xTaskCreatePinnedToCore(guiProcess, "gui", configMINIMAL_STACK_SIZE * 32, NULL, 0, NULL, guiCpuCore); 
}
