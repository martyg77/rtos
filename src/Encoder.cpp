#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "Encoder.h"

void encoderGpioHandler(void* p) {
    TaskHandle_t* t = (TaskHandle_t*) p;
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE; // Not sure what this does
    vTaskNotifyGiveFromISR(*t, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(); // Not sure what this does
}

typedef struct {
    gpio_num_t pinA;
    gpio_num_t pinB;
} encoderProcess_t;

void encoderProcess(void* p) {
    encoderProcess_t* tcb = (encoderProcess_t*) p;
    
    // My KY-040 module has 3 pullups
    gpio_pad_select_gpio(tcb->pinA); // PinMux magic
    gpio_pad_select_gpio(tcb->pinB); // PinMux magic
    gpio_config_t g = {
        .pin_bit_mask = (1ull << tcb->pinA) | (1ull << tcb->pinB),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE};        
    gpio_config(&g);

    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    gpio_isr_handler_add(tcb->pinA, encoderGpioHandler, &task);
    gpio_isr_handler_add(tcb->pinB, encoderGpioHandler, &task);

    // Main loop
    uint32_t rc;
    while (true) {
        rc = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("(%i) Encoder %i\n", xTaskGetTickCount(), rc); 
    }

    // We should never reach this point
    vTaskDelete(NULL);
}

Encoder::Encoder(gpio_num_t pinCLK, gpio_num_t pinDT) {
    // These two signals are interchangable
    pinA = pinCLK; 
    pinB = pinDT;
    tcb = new(encoderProcess_t) { pinA, pinB };
    xTaskCreate(encoderProcess, "encoder", configMINIMAL_STACK_SIZE * 4, tcb, 0, &task);
}

Encoder::~Encoder() {
    delete((encoderProcess_t*) tcb);
    vTaskDelete(task);
}
