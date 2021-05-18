// KY-040 rotary encoder wheel section

#ifndef ENCODER_H
#define ENCODER_H

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Encoder {
  public:
    Encoder(const gpio_num_t pinA, const gpio_num_t pinB);
    ~Encoder();

    int value(); // Current encoder value
    int delta(); // Encode value difference between previous delta() call and now

  private:
    gpio_num_t pinA = GPIO_NUM_NC;
    gpio_num_t pinB = GPIO_NUM_NC;
    TaskHandle_t task = nullptr;
    void *tcb = nullptr;
};

#endif // ENCODER_H
