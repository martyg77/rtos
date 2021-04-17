#include <freertos/task.h>
#include <driver/gpio.h>

class Button
{
public:
    Button(gpio_num_t pin);
    ~Button();

private:
    gpio_num_t pin = GPIO_NUM_NC;
    TaskHandle_t handle = nullptr;
};
