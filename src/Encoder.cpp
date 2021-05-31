#include "Encoder.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdio.h>

typedef struct {
    gpio_num_t pinA;
    gpio_num_t pinB;
    QueueHandle_t queue;
    int16_t value;
    int16_t deltaPrev;
} encoderProcess_t;

void encoderGpioHandler(encoderProcess_t *tcb) {
    uint8_t ab = (gpio_get_level(tcb->pinA) << 1) | gpio_get_level(tcb->pinB);
    xQueueSendFromISR(tcb->queue, &ab, nullptr);
}

void encoderProcess(encoderProcess_t *tcb) {
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
    ESP_ERROR_CHECK(gpio_isr_handler_add(tcb->pinA, (gpio_isr_t)encoderGpioHandler, tcb));
    ESP_ERROR_CHECK(gpio_isr_handler_add(tcb->pinB, (gpio_isr_t)encoderGpioHandler, tcb));

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
    vTaskDelete(nullptr);
}

int Encoder::value() { return ((encoderProcess_t *)tcb)->value; };

int Encoder::delta() {
    encoderProcess_t *p = (encoderProcess_t *)tcb;
    int d = p->value - p->deltaPrev;
    p->deltaPrev = p->value;
    return d;
};

Encoder::Encoder(const gpio_num_t pinCLK, const gpio_num_t pinDT) {
    // These two signals are interchangable
    pinA = pinCLK;
    pinB = pinDT;
    tcb = new (encoderProcess_t){pinA, pinB, nullptr, 0, 0};
    xTaskCreate((TaskFunction_t)encoderProcess, "encoder", configMINIMAL_STACK_SIZE * 4, tcb, 0, &task);
}

Encoder::~Encoder() {
    delete ((encoderProcess_t *)tcb);
    vTaskDelete(task);
}
