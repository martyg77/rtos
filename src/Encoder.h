#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

// KY-040 rotary encoder wheel section

class Encoder
{
public:
    Encoder(gpio_num_t pinA, gpio_num_t pinB);
    ~Encoder();

private:
    gpio_num_t pinA = GPIO_NUM_NC;
    gpio_num_t pinB = GPIO_NUM_NC;
    TaskHandle_t task = NULL;
    void* tcb = NULL;
};
