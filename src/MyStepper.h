// TMC2209 controller, no-name 17HS4401 NEMA-17 stepper motor

#ifndef MYSTEPPER_H
#define MYSTEPPER_H

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>

class MyStepper {
  public:
    MyStepper(const gpio_num_t en, const gpio_num_t dir, const gpio_num_t step);
    ~MyStepper();

    void clockwise();
    void counterclockwise();
    void reverse();

    void step();

    void demo(); // TODO This procedure never returns

  private:
    gpio_num_t EN = GPIO_NUM_NC;
    gpio_num_t DIR = GPIO_NUM_NC;
    gpio_num_t STEP = GPIO_NUM_NC;

    bool clockwiseRotation = false; // TODO Use signed velocity and eliminate
    void setRotation(const bool clockwise);
};

#endif // MYSTEPPER_H
