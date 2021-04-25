#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>

// TMC2209 controller, no-name 17HS4401 NEMA-17 stepper motor

class MyStepper
{
public:
    MyStepper(gpio_num_t en, gpio_num_t dir, gpio_num_t step);
    ~MyStepper();

    void clockwise();
    void counterclockwise();
    void reverse();

    void step();

private:
    gpio_num_t EN = GPIO_NUM_NC;
    gpio_num_t DIR = GPIO_NUM_NC;
    gpio_num_t STEP = GPIO_NUM_NC;
    
    bool clockwiseRotation = false; // TODO Use signed velocity and eliminate
    void setRotation(bool clockwise);
};
