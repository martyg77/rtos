// Momentary pushbutton switch, normally open, active low
// As seen on KY-040 rotary encoder

#ifndef BUTTON_H
#define BUTTON_H

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

class Button {
  public:
    Button(const gpio_num_t pin);
    ~Button();

    // Button state must be stable for this long to report press/release
    const static int debounce_mS = 50;

    bool pressed(); // Current button state (polling API)
    // TODO callback based interface

  private:
    gpio_num_t pin = GPIO_NUM_NC;
    TaskHandle_t task = nullptr;
    void *tcb = nullptr;
};

#endif // BUTTON_H
