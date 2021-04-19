#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include "Encoder.h"

typedef struct {
    gpio_num_t pinA;
    gpio_num_t pinB;
    QueueHandle_t queue;
    int16_t value;
} encoderProcess_t;

void encoderGpioHandler(void* p) {
    encoderProcess_t* tcb = (encoderProcess_t*) p;
    uint8_t ab = (gpio_get_level(tcb->pinA) << 1) | gpio_get_level(tcb->pinB);
    xQueueSendFromISR(tcb->queue, &ab, NULL);
}

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

    tcb->queue = xQueueCreate(16, sizeof(uint8_t));
    gpio_isr_handler_add(tcb->pinA, encoderGpioHandler, tcb);
    gpio_isr_handler_add(tcb->pinB, encoderGpioHandler, tcb);

    uint8_t state = 0;
    while (true) {
        uint8_t ab;
        xQueueReceive(tcb->queue, &ab, portMAX_DELAY);
        state = ((state << 2) | (ab & 0x3)) & 0xf;
        static const int8_t quadratureTable[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
        tcb->value += quadratureTable[state];
        printf("(%i) Encoder %x %x %i\n", xTaskGetTickCount(), ab, state, tcb->value);
    }

    // We should never reach this point
    vQueueDelete(tcb->queue);
    vTaskDelete(NULL);
}

Encoder::Encoder(gpio_num_t pinCLK, gpio_num_t pinDT) {
    // These two signals are interchangable
    pinA = pinCLK; 
    pinB = pinDT;
    tcb = new(encoderProcess_t) { pinA, pinB, NULL, 0 };
    xTaskCreate(encoderProcess, "encoder", configMINIMAL_STACK_SIZE * 4, tcb, 0, &task);
}

Encoder::~Encoder() {
    delete((encoderProcess_t*) tcb);
    vTaskDelete(task);
}