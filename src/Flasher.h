// Flash a LED connected to a GPIO pin - How hard can this be?

#ifndef FLASHER_H
#define FLASHER_H

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Flasher {
  public:
    Flasher(const gpio_num_t pin, const uint speed);
    ~Flasher();

    // TODO specify cadence and duty-cycle, currently 1 second 33% on

  private:
    gpio_num_t pin = GPIO_NUM_NC;
    int speed = 0;
    TaskHandle_t handle = nullptr;
    void *tcb = nullptr;
};

#endif // FLASHER_H
