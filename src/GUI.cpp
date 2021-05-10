#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h" // TODO order sensitive

#include "lvgl.h"
#include "lvgl_helpers.h"
#include "lv_examples/src/lv_demo_widgets/lv_demo_widgets.h"
#include "lv_examples/src/lv_demo_keypad_encoder/lv_demo_keypad_encoder.h"
#include "GUI.h"

#include "Button.h"
#include "Encoder.h"

Button ky040_button(GPIO_NUM_25);
Encoder ky040_encoder(GPIO_NUM_26, GPIO_NUM_27);
lv_indev_t* ky040_device = NULL; // TODO hacked global for lv_demo_keypad_encoder

bool guiEncoderRead(lv_indev_drv_t* p, lv_indev_data_t* d) {
//  d->point = {0, 0};
//  d->key = 0;
//  d->btn_id = 0;
    d->enc_diff = ky040_encoder.delta();
    d->state = (ky040_button.pressed()) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    return false; // No buffering support
}

// LVGL 10mS tick timebase, called directly from rtos timer interrupt
bool timerCallBack(void) {
    lv_tick_inc(10);
    return true;
}

// GUI Task
void guiProcess(void* p) {
    // Allocate DMA-capable memory for display double-buffering
    lv_color_t* buf1 = (lv_color_t*)heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t* buf2 = (lv_color_t*)heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);
    assert(buf2 != NULL);

    // Initialize LVGL susbsystem
    lv_init();
    lv_disp_buf_t buffer;
    lv_disp_buf_init(&buffer, buf1, buf2, DISP_BUF_SIZE);
    lv_disp_drv_t display;
    lv_disp_drv_init(&display);
    display.buffer = &buffer;
    display.flush_cb = disp_driver_flush;
    lv_disp_drv_register(&display);

    // Output (video) driver
    lvgl_driver_init();

    // Rotary encoder for input
    lv_indev_drv_t encoder;
    lv_indev_drv_init(&encoder);
    encoder.type = LV_INDEV_TYPE_ENCODER;
    encoder.read_cb = guiEncoderRead;
    encoder.feedback_cb = NULL;
    ky040_device = lv_indev_drv_register(&encoder); // TODO hacked into lv_demo_keypad_encoder

    // User application
//  lv_demo_widgets();
    lv_demo_keypad_encoder();

    // LVGL tick timer, runs every RTOS tick (10mS) at high priority
    esp_register_freertos_idle_hook_for_cpu(timerCallBack, guiCpuCore);

    // Main loop, executes forever based on hint returned from lv_task_handler()
    while (true) { vTaskDelay(lv_task_handler() / portTICK_PERIOD_MS); }

    // We should never reach this point
    free(buf1);
    free(buf2);
    vTaskDelete(NULL);
}
