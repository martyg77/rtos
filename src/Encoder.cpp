#include "Encoder.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdio.h>

void encoderGpioISR(Encoder *tcb) {
    uint8_t ab = (gpio_get_level(tcb->pinA) << 1) | gpio_get_level(tcb->pinB);
    BaseType_t wake = pdFALSE;
    xQueueSendFromISR(tcb->queue, &ab, &wake);
    if (wake == pdTRUE) portYIELD_FROM_ISR(); // Request context switch
}

void encoderProcess(Encoder *tcb) {
    gpio_pad_select_gpio(tcb->pinA);
    gpio_pad_select_gpio(tcb->pinB);
    gpio_config_t g = {
        .pin_bit_mask = (1ull << tcb->pinA) | (1ull << tcb->pinB),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE};
    gpio_config(&g);

    tcb->queue = xQueueCreate(16, sizeof(uint8_t));
    gpio_isr_handler_add(tcb->pinA, (gpio_isr_t)encoderGpioISR, tcb);
    gpio_isr_handler_add(tcb->pinB, (gpio_isr_t)encoderGpioISR, tcb);

    // Process received state changes into tally counter
    uint8_t state = 0;
    while (true) {
        uint8_t ab;
        xQueueReceive(tcb->queue, &ab, portMAX_DELAY);
        state = ((state << 2) | (ab & 0x3)) & 0xf;
        static const int8_t quadratureTable[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
        tcb->tally += quadratureTable[state];
        // printf("(%i) Encoder %x %x %i\n", xTaskGetTickCount(), ab, state, tcb->tally);
    }

    // We should never reach this point
    vQueueDelete(tcb->queue);
    vTaskDelete(nullptr);
}

int Encoder::value() { return tally; };

int Encoder::delta() {
    int d = tally - deltaPrev;
    deltaPrev = tally;
    return d;
};

Encoder::Encoder(const gpio_num_t pinCLK, const gpio_num_t pinDT) {
    // These two signals are interchangable, direction will be reversed
    pinA = pinCLK;
    pinB = pinDT;
    xTaskCreate((TaskFunction_t)encoderProcess, "encoder", configMINIMAL_STACK_SIZE * 4, this, 0, &task);
}

Encoder::~Encoder() {
    vTaskDelete(task);
}
