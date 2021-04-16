#include <driver/gpio.h>

class Flasher
{
public:
    Flasher(gpio_num_t, int);

private:
    gpio_num_t led = GPIO_NUM_NC;
    int speed = 0;
};
