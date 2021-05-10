#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

// Momentary pushbutton switch, normally open, active low
// As seen on KY-040 rotary encoder

class Button {
public:
    Button(gpio_num_t pin);
    ~Button();

    const static int debounce_mS = 50;

    bool pressed(); // Current button state (polling API)
    // TODO callback based interface

private:
    gpio_num_t pin = GPIO_NUM_NC;
    TaskHandle_t task = NULL;
    void* tcb = NULL;
};