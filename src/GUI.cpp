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
#include "SSD1306.h"
#include "Button.h"
#include "Encoder.h"

// Encoder and button input, feedback to screen
// Real-time data output
// Basic LVGL scheduling (timer)
// Async events from other tasks
// Banner can just scroll a text string for now
// GUI is all-knowing application-aware, should be topmost on include tree, coupled to application class
// Could be the same as main task, but need to control affinity, priority, etc.

lv_obj_t *scr= NULL;
lv_obj_t *banner= NULL;
lv_obj_t *body= NULL;
lv_obj_t *accelX= NULL;
lv_obj_t *accelY= NULL;
lv_obj_t *accelZ= NULL;
lv_obj_t *gyroX= NULL;
lv_obj_t *gyroY= NULL;
lv_obj_t *gyroZ= NULL;
lv_obj_t *obj = NULL;

void statusScreen() {
    scr = lv_obj_create(NULL, NULL);
    lv_scr_load(scr);
   
    obj = lv_obj_create(scr, NULL);
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 128, 16);
    banner = lv_label_create(obj, NULL);
    lv_label_set_long_mode(banner, LV_LABEL_LONG_SROLL_CIRC);
    lv_obj_set_width(banner, 128);
    lv_label_set_anim_speed(banner, 10); // pixels/sec
    lv_label_set_text(banner, "The quick brown fox jumped over the lazy dog");
    lv_obj_align(banner, obj, LV_ALIGN_CENTER, 0, 0);

    body = lv_obj_create(scr, NULL);
    lv_obj_set_pos(body, 0, 16);
    lv_obj_set_size(body, 128, 48);

    obj = lv_obj_create(body, NULL);
    lv_obj_set_size(obj, 42, 24);
    lv_obj_align(obj, body, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    accelX = lv_label_create(obj, NULL);
    lv_obj_align(accelX, obj, LV_ALIGN_CENTER, 0, 0);

    obj = lv_obj_create(body, obj);
    lv_obj_align(obj, body, LV_ALIGN_IN_TOP_MID, 0, 0);
    accelY = lv_label_create(obj, NULL);
    lv_obj_align(accelY, obj, LV_ALIGN_CENTER, 0, 0);

    obj = lv_obj_create(body, obj);
    lv_obj_align(obj, body, LV_ALIGN_IN_TOP_RIGHT, 0, 0);
    accelZ = lv_label_create(obj, NULL);
    lv_obj_align(accelZ, obj, LV_ALIGN_CENTER, 0, 0);
    
    obj = lv_obj_create(body, obj);
    lv_obj_align(obj, body, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);
    gyroX = lv_label_create(obj, NULL);
    lv_obj_align(gyroX, obj, LV_ALIGN_CENTER, 0, 0);
   
    obj = lv_obj_create(body, obj);
    lv_obj_align(obj, body, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
    gyroY = lv_label_create(obj, NULL);
    lv_obj_align(gyroY, obj, LV_ALIGN_CENTER, 0, 0);

    obj = lv_obj_create(body, obj);
    lv_obj_align(obj, body, LV_ALIGN_IN_BOTTOM_RIGHT, 0, 0);
    gyroZ = lv_label_create(obj, NULL);
    lv_obj_align(gyroZ, obj, LV_ALIGN_CENTER, 0, 0);

    lv_label_set_text(accelX, "111");
    lv_label_set_text(accelY, "567");
    lv_label_set_text(accelZ, "-61");
    lv_label_set_text(gyroX, "320");
    lv_label_set_text(gyroY, "-111");
    lv_label_set_text(gyroZ, "9");
}

// LVGL 10mS tick timebase, called directly from rtos timer interrupt

bool timerCallBack(void) {
    lv_tick_inc(10);
    return true;
}

Encoder *encoder = NULL;
Button *button = NULL;

bool guiEncoderRead(lv_indev_drv_t* p, lv_indev_data_t* d) {
//  d->point = {0, 0};
//  d->key = 0;
//  d->btn_id = 0;
    d->enc_diff = encoder->delta();
    d->state = (button->pressed()) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    return false; // No buffering support
}

// GUI Task

void guiProcess(void* p) {

    SSD1306 oled(GPIO_NUM_21, GPIO_NUM_22);
    Encoder ky040_encoder(GPIO_NUM_26, GPIO_NUM_27);
    Button ky040_button(GPIO_NUM_25);
    encoder = &ky040_encoder;
    button = &ky040_button;

    // Rotary encoder with pushbutton for input
    lv_indev_drv_t d;
    lv_indev_drv_init(&d);
    d.type = LV_INDEV_TYPE_ENCODER;
    d.read_cb = guiEncoderRead;
    d.feedback_cb = NULL;
    lv_indev_t *indev = lv_indev_drv_register(&d);

    // Output screen objects
    statusScreen();
   
    lv_group_t *g = lv_group_create();
    lv_group_add_obj(g, accelX);
    lv_group_add_obj(g, accelY);
    lv_group_add_obj(g, accelZ);
    lv_group_add_obj(g, gyroX);
    lv_group_add_obj(g, gyroY);
    lv_group_add_obj(g, gyroZ);
    lv_indev_set_group(indev, g);
 
    // LVGL tick timer, runs every RTOS tick (10mS) at high priority
    esp_register_freertos_idle_hook_for_cpu(timerCallBack, guiCpuCore);

    // Main loop, executes forever based on hint returned from lv_task_handler()
    while (true) { vTaskDelay(lv_task_handler() / portTICK_PERIOD_MS); }

    // We should never reach this point
    vTaskDelete(NULL);
}
