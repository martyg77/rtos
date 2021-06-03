// Differential encoder support
// e.g. KY-040 rotary encoder rotary knob
// e.g. AsLong JGB37-520B-12v-178RPM DC motor, 56:1, 11-pole hall-effect encoder

// This version leverages ESP32 PCNT pulse counters
// Ref. https://github.com/espressif/esp-idf/tree/master/examples/peripherals/pcnt

#pragma once

#include <driver/gpio.h>
#include <driver/pcnt.h>

// Call this procedure before creating any ESP32Encode objects
void esp32_encoder_install();

class ESP32Encoder {
  public:

    ESP32Encoder(gpio_num_t pinA, gpio_num_t pinB, pcnt_unit_t unit);

    void setCount(int64_t i) { tally = i - getHWCount(); }
    int64_t getCount() { return tally + getHWCount(); }
    void clearCount();
    int64_t pauseCount() { return pcnt_counter_pause(unit); }
    int64_t resumeCount() { return pcnt_counter_resume(unit); }

    long value() { return getCount(); }
    int delta();

    // TODO these members should be private

    pcnt_unit_t unit = PCNT_UNIT_MAX;
    int16_t getHWCount();
    volatile int64_t tally = 0; // Updated by overflow handler
    static const int counterMAX = 32000; // Somewhere near under INT16_MAX for efficiency

    static ESP32Encoder *encoders[PCNT_UNIT_MAX];

  private:
    gpio_num_t pinA = GPIO_NUM_NC;
    gpio_num_t pinB = GPIO_NUM_NC;

    uint64_t tallyPrev = 0;

    void setFilter(uint16_t value); // APB_CLK pulses, 10-bit register
};
