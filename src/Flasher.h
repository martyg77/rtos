#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Flasher
{
public:
    Flasher(gpio_num_t pin, int speed);
    ~Flasher();

private:
    gpio_num_t pin = GPIO_NUM_NC;
    int speed = 0;
    TaskHandle_t handle = NULL;
    void* tcb = NULL;
};
