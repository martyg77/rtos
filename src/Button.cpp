#include "Button.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <stdio.h>

typedef struct {
    gpio_num_t pin;
    bool state;
} buttonProcess_t;

static void buttonTimerHandler(TaskHandle_t* t) {
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;  // Not sure what this does
    vTaskNotifyGiveFromISR(*t, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR();  // Not sure what this does
}

void buttonTimerCallBack(TimerHandle_t x) {
    TaskHandle_t* t = (TaskHandle_t*)pvTimerGetTimerID(x);
    xTaskNotifyGive(*t);
}

void buttonProcess(buttonProcess_t* tcb) {
    // My KY-040 module has 3 pullups
    // Switch signal is active low
    // We only care about press, not release or persistent state
    // TODO Incorrect - LVGL will poll for pressed/released
    gpio_pad_select_gpio(tcb->pin);  // PinMux magic
    gpio_config_t g = {
        .pin_bit_mask = (1ull << tcb->pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE};
    gpio_config(&g);

    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    gpio_isr_handler_add(tcb->pin, (gpio_isr_t)buttonTimerHandler, &task);
    static const TickType_t debounceTicks = Button::debounce_mS / portTICK_PERIOD_MS;
    TimerHandle_t timer = xTimerCreate("button", debounceTicks, pdFALSE, &task, buttonTimerCallBack);

    // Debouncer loop, notified on GPIO interrupt or timer expiry
    while (true) {
        // Wait for first state change
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        do { // Ensure input stays stable for at least the debounce timer interval
            xTimerReset(timer, debounceTicks);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        } while (xTimerIsTimerActive(timer));

        tcb->state = !gpio_get_level(tcb->pin);
        printf("(%i) Button %i\n", xTaskGetTickCount(), tcb->state);
    }

    // We should never reach this point
    xTimerDelete(timer, 0);
    vTaskDelete(NULL);
}

bool Button::pressed() { return ((buttonProcess_t*) tcb)->state; }

Button::Button(gpio_num_t p) {
    pin = p;
    tcb = new(buttonProcess_t) { pin, 0 };
    xTaskCreate((TaskFunction_t)buttonProcess, "button", configMINIMAL_STACK_SIZE * 4, tcb, 0, &task);
}

Button::~Button() {
    delete((buttonProcess_t*)tcb);
    vTaskDelete(task);
}
