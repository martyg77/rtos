#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include "Button.hpp"

typedef struct {
    gpio_num_t pin;
    QueueHandle_t q;
} handler_config;

static void handler(void* p) {
    handler_config* x = (handler_config*) p;
    int i = gpio_get_level(x->pin);
    xQueueSendFromISR(x->q, &i, nullptr);
}
 
// void Button::task(void* p) {}
void task(void* p) {
    gpio_num_t* pin = (gpio_num_t*) p;
    
    // My KY-040 module has 3 pullups
    // Switch signal is active low
    // We only care about press, not release or persistent state
    gpio_config_t g = {
        .pin_bit_mask = (1ull << *pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE};        
    gpio_config(&g);

    QueueHandle_t q = xQueueCreate(10, sizeof(int));
    handler_config x = {*pin, q};
    gpio_isr_handler_add(*pin, handler, &x);

    while (true) {
        int i;
        xQueueReceive(q, &i, portMAX_DELAY);
        printf("(%i) Received %i \n", xTaskGetTickCount(), i);

        // (re)start the timer, when it pops after 50mS we have a stable signal
     //   this->callback();
    }

    vTaskDelete(NULL);
}

Button::Button(gpio_num_t pin)
{
    this->pin = pin;
    xTaskCreate(task, "button", configMINIMAL_STACK_SIZE * 4, &this->pin, 0, &this->handle);
}

void Button::install_callback(ButtonFunction_t p) { this->callback = p; }

void Button::wait_for_press() {}

Button::~Button() {
    vTaskDelete(this->handle);
}
