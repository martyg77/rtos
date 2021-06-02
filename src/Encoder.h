// Differential encoder support
// e.g. KY-040 rotary encoder rotary knob
// e.g. AsLong JGB37-520B-12v-178RPM DC motor, 56:1, 11-pole hall-effect encoder

#pragma once

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

class Encoder {
  public:
    Encoder(const gpio_num_t pinA, const gpio_num_t pinB);
    ~Encoder();

    long value(); // Current accumulated encoder value since constructed
    int delta(); // Value difference between previous delta() call and most recent tally

    gpio_num_t pinA = GPIO_NUM_NC;
    gpio_num_t pinB = GPIO_NUM_NC;
    TaskHandle_t task = nullptr;
    QueueHandle_t queue = nullptr;
    long samples = 0;
    long tally = 0;
    long tallyPrev = 0;
};
