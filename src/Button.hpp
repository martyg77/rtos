#include <freertos/task.h>
#include <driver/gpio.h>

// Momentary pushbutton switch, normally open, active low
// As seen on KY-040 rotary encoder

class Button
{
public:
    Button(gpio_num_t pin);

    const int debounce_mS = 50;

    // Function to call on button press occurence; runs on Button class thread
    typedef void (*ButtonFunction_t)(void*);
    void install_callback(ButtonFunction_t p);

    // Blocks caller until button press occurence
    void wait_for_press();

    ~Button();

private:
    // void task(void* p);
    gpio_num_t pin = GPIO_NUM_NC;
    TaskHandle_t handle = nullptr;
    ButtonFunction_t callback = NULL;
};
