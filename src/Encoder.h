// Differential encoder support
// e.g. KY-040 rotary encoder rotary knob
// e.g. AsLong JGB37-520B-333RPM DC motor, 13-pole hall-effect encoder

#pragma once

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

class Encoder {
  public:
    Encoder(const gpio_num_t pinA, const gpio_num_t pinB);
    ~Encoder();

    int value(); // Current accumulated encoder value since constructed
    int delta(); // Encode value difference between previous delta() call and most recent tally

    gpio_num_t pinA = GPIO_NUM_NC;
    gpio_num_t pinB = GPIO_NUM_NC;
    TaskHandle_t task = nullptr;
    QueueHandle_t queue = nullptr;
    int16_t tally = 0;
    int16_t deltaPrev = 0;
};
