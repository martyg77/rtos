#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include <esp32/rom/ets_sys.h>
#include <driver/gpio.h>
#include "MyStepper.h"

// Ref. TMS2209 datasheet, section 13, "STEP/DIR Interface"
// CLK pin strapped low for internal clocking (fclk = 12MHz, tclk = 83.3nS)

void MyStepper::setRotation(bool r) {
    gpio_set_level(DIR, r);
    clockwiseRotation = r;
    // ets_delay_us(1); // tDSU = 20ns << 1uS
}

void MyStepper::clockwise() { setRotation(true); }
void MyStepper::counterclockwise() { setRotation(false); }
void MyStepper::reverse() { setRotation(!clockwiseRotation);  }

void MyStepper::step() {
    gpio_set_level(STEP, 1);
    ets_delay_us(1); // tSH = tCLK + 20 = 104nS << 1uS
    gpio_set_level(STEP, 0);
    // ets_delay_us(1); // tSL = tCLK + 20 = 104nS << 1uS
}

MyStepper::MyStepper(gpio_num_t e, gpio_num_t d, gpio_num_t s) {
    EN = e;
    DIR = d;
    STEP = s;

    gpio_pad_select_gpio(EN);
    gpio_pad_select_gpio(DIR);
    gpio_pad_select_gpio(STEP);

    gpio_config_t g = {
        .pin_bit_mask = (1ull << EN) | (1ull << DIR) | (1ull << STEP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};        
    gpio_config(&g);

    gpio_set_level(STEP, 0);
    setRotation(clockwiseRotation);
    gpio_set_level(EN, 0); // Enable motor output power stage
}

MyStepper::~MyStepper() {}
