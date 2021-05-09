#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h" // TODO order sensitive

#define LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#include "lvgl_helpers.h"
#include "lv_examples/src/lv_demo_widgets/lv_demo_widgets.h"
#include "GUI.h"

// LVGL 10mS tick timebase, called directly from rtos timer interrupt
bool timerCallBack(void) {
    lv_tick_inc(10);
    return true;
}

// Called by gui gui request input
bool guiEncoderRead(lv_indev_drv_t* p, lv_indev_data_t* d) {
    d->point = {0, 0};
    d->key = 0;
    d->btn_id = 0;
    d->enc_diff = 0;
    d->state = 0;

    return false; // No additonal data to be read
}

// Called by gui to notify something happened, give feedback opportunity
void guiEncoderFeedback(lv_indev_drv_t* p, lv_event_t event) {}

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
    encoder.read_cb = NULL;      // (struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
    encoder.feedback_cb = NULL;  // (struct _lv_indev_drv_t *, uint8_t);
    lv_indev_drv_register(&encoder);

    // User application
    lv_demo_widgets();

    // LVGL tick timer, runs every RTOS tick (10mS) at high priority
    esp_register_freertos_idle_hook_for_cpu(timerCallBack, guiCpuCore);

    // Main loop, executes forever based on hint returned from lv_task_handler()
    while (true) {
        vTaskDelay(lv_task_handler() / portTICK_PERIOD_MS);
    }

    // We should never reach this point
    free(buf1);
    free(buf2);
    vTaskDelete(NULL);
}
