#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class Flasher
{
public:
    typedef struct {
        gpio_num_t pin;
        int speed;
    } flasherConfig;

    Flasher(gpio_num_t pin, int speed);
    ~Flasher();

private:
    flasherConfig config = { GPIO_NUM_NC, 0 };
    TaskHandle_t handle = NULL;
};
