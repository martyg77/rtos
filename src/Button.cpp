#include "Button.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <stdio.h>

static void timerHandler(void* p) {
    TaskHandle_t* t = (TaskHandle_t*)p;
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;  // Not sure what this does
    vTaskNotifyGiveFromISR(*t, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR();  // Not sure what this does
}

void timerCallBack(TimerHandle_t x) {
    TaskHandle_t* t = (TaskHandle_t*)pvTimerGetTimerID(x);
    xTaskNotifyGive(*t);
}

void taskFunction(void* p) {
    gpio_num_t* pin = (gpio_num_t*)p;

    // My KY-040 module has 3 pullups
    // Switch signal is active low
    // We only care about press, not release or persistent state
    gpio_pad_select_gpio(*pin);  // PinMux magic
    gpio_config_t g = {
        .pin_bit_mask = (1ull << *pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE};
    gpio_config(&g);

    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    gpio_isr_handler_add(*pin, timerHandler, &task);
    static const TickType_t timerTicks = Button::debounce_mS / portTICK_PERIOD_MS;
    TimerHandle_t timer = xTimerCreate("button", timerTicks, pdFALSE, &task, timerCallBack);

    // Debouncer loop, notified on GPIO interrupt or timer expiry
    while (true) {
        printf("(%i) Received\n", xTaskGetTickCount());
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        do {
            xTimerReset(timer, timerTicks);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        } while (xTimerIsTimerActive(timer));

        if (!gpio_get_level(*pin))  // Driven low for debounce interval
            printf("(%i) Debounced\n", xTaskGetTickCount());
    }

    // We should never reach this point
    xTimerDelete(timer, 0);
    vTaskDelete(NULL);
}

Button::Button(gpio_num_t p) {
    pin = p;
    xTaskCreate(taskFunction, "button", configMINIMAL_STACK_SIZE * 4, &this->pin, 0, &this->task);
}

Button::~Button() {
    vTaskDelete(task);
}
