#include <driver/gpio.h>

#include "Button.h"
#include "Encoder.h"
#include "Flasher.h"
#include "MyStepper.h"

const gpio_num_t GPIO_LED_RED = GPIO_NUM_0;
const gpio_num_t GPIO_LED_GREEN = GPIO_NUM_2;
const gpio_num_t GPIO_LED_BLUE = GPIO_NUM_4;

// Flasher red(GPIO_LED_RED, 250);
Flasher green(GPIO_LED_GREEN, 300);
// Flasher blue(GPIO_LED_BLUE, 500);

// Button ky040_button(GPIO_NUM_25);
// Encoder ky040_encoder(GPIO_NUM_26, GPIO_NUM_27);
// Button nil_button(GPIO_NUM_39);

MyStepper stepper(GPIO_NUM_5, GPIO_NUM_19, GPIO_NUM_18);

const int motorSteps = 200;                                                  // 200 steps/revolution
const int motorMicroSteps = 8;                                               // 8 microsteps/step (MS[01] straps)
const int motorRPM = 120;                                                    // current motor speed
const int motorStepVelocity = motorRPM * motorSteps * motorMicroSteps / 60;  // current microsteps/sec, not to exceed fCLK/512 = 23400

extern "C" {
void app_main();
}

void app_main() {
    // This procedure can only be called once
    gpio_install_isr_service(0);  // TODO Needs to run *BEFORE* constuctors!

    // Demo loop: 5 revolutions, stop, reverse, repeat
    stepper.clockwise();
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        for (int i = 0; i < 5 * motorSteps * motorMicroSteps; i++) {
            stepper.step();
            ets_delay_us(1000000 / motorStepVelocity);  // uS between (evenly spaced) processor step pulses
        }
        stepper.reverse();
    }
}
