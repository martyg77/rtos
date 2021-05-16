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
#include "ILI9341.h"

// LVGL internal handles
Encoder *encoder = NULL;
Button *button = NULL;
lv_indev_t* ky040_device = NULL; // TODO hacked global for lv_demo_keypad_encoder

// Encoder and button input, feedback to screen
// Real-time data output
// Basic LVGL scheduling (timer) for lv_task_handler()
// Async events from other tasks
// GUI is all-knowing application-aware, should be topmost on include tree, coupled to application class

// Update values on screen by updating these objects from the GUI task timeslice
lv_obj_t *accelX = NULL;
lv_obj_t *accelY = NULL;
lv_obj_t *accelZ = NULL;
lv_obj_t *gyroX = NULL;
lv_obj_t *gyroY = NULL;
lv_obj_t *gyroZ = NULL;

bool guiEncoderRead(lv_indev_drv_t* p, lv_indev_data_t* d) {
//  d->point = {0, 0};
//  d->key = 0;
//  d->btn_id = 0;
    d->enc_diff = encoder->delta();
    d->state = (button->pressed()) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    return false; // No buffering support
}

static void guiEncoderEvent(lv_obj_t * obj, lv_event_t event) {
    printf("[0x%08x] ", (uint32_t) obj);
    switch (event) {
    case LV_EVENT_PRESSED:
        printf("Pressed\n");
        break;

    case LV_EVENT_RELEASED:
        printf("Released\n");
        break;

    case LV_EVENT_KEY:
        printf("Key\n");
        break;

    case LV_EVENT_FOCUSED: // TODO highlight object border while focused -- style tweaks?
        printf("Focus\n");
        break;

    case LV_EVENT_DEFOCUSED:
        printf("Defocus\n");
        break;

    default:
        printf("Unknown %i\n", event);
    }
}

void statusScreen() {
    lv_obj_t *scr = lv_obj_create(NULL, NULL);
    lv_scr_load(scr);
   
    lv_obj_t *obj = lv_obj_create(scr, NULL);
    lv_obj_set_pos(obj, 0, 0);
    lv_obj_set_size(obj, 128, 16);
    lv_obj_t *banner = lv_label_create(obj, NULL);
    lv_label_set_long_mode(banner, LV_LABEL_LONG_SROLL_CIRC);
    lv_obj_set_width(banner, 128);
    lv_label_set_anim_speed(banner, 10); // pixels/sec
    lv_label_set_text(banner, "The quick brown fox jumped over the lazy dog");
    lv_obj_align(banner, obj, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *body = lv_obj_create(scr, NULL);
    lv_obj_set_pos(body, 0, 16);
    lv_obj_set_size(body, 128, 48);

    obj = lv_obj_create(body, NULL);
    lv_obj_set_size(obj, 42, 24);
    lv_obj_align(obj, body, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    accelX = lv_label_create(obj, NULL);
    lv_obj_align(accelX, obj, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_align(accelX, LV_LABEL_ALIGN_RIGHT); // TODO this does not work
    lv_obj_set_event_cb(accelX, guiEncoderEvent);

    obj = lv_obj_create(body, obj);
    lv_obj_align(obj, body, LV_ALIGN_IN_TOP_MID, 0, 0);
    accelY = lv_label_create(obj, NULL);
    lv_obj_align(accelY, obj, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(accelY, guiEncoderEvent);

    obj = lv_obj_create(body, obj);
    lv_obj_align(obj, body, LV_ALIGN_IN_TOP_RIGHT, 0, 0);
    accelZ = lv_label_create(obj, NULL);
    lv_obj_align(accelZ, obj, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(accelZ, guiEncoderEvent);
    
    obj = lv_obj_create(body, obj);
    lv_obj_align(obj, body, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);
    gyroX = lv_label_create(obj, NULL);
    lv_obj_align(gyroX, obj, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(gyroX, guiEncoderEvent);
   
    obj = lv_obj_create(body, obj);
    lv_obj_align(obj, body, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
    gyroY = lv_label_create(obj, NULL);
    lv_obj_align(gyroY, obj, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(gyroY, guiEncoderEvent);

    obj = lv_obj_create(body, obj);
    lv_obj_align(obj, body, LV_ALIGN_IN_BOTTOM_RIGHT, 0, 0);
    gyroZ = lv_label_create(obj, NULL);
    lv_obj_align(gyroZ, obj, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(gyroZ, guiEncoderEvent);
   
    // Event group coupled to encoder callback
    lv_group_t *g = lv_group_create();
    lv_group_add_obj(g, accelX);
    lv_group_add_obj(g, accelY);
    lv_group_add_obj(g, accelZ);
    lv_group_add_obj(g, gyroX);
    lv_group_add_obj(g, gyroY);
    lv_group_add_obj(g, gyroZ);
    lv_label_set_align(gyroZ, LV_LABEL_ALIGN_RIGHT); // TODO this does not work
    lv_indev_set_group(ky040_device, g);

    lv_label_set_text_fmt(accelX, "%i", 111);
    lv_label_set_text_fmt(accelY, "%i", 567);
    lv_label_set_text_fmt(accelZ, "%i", -61);
    lv_label_set_text_fmt(gyroX, "%i", 320);
    lv_label_set_text_fmt(gyroY, "%i", -111);
    lv_label_set_text_fmt(gyroZ, "%i", 9);
}

// LVGL 10mS tick timebase, called directly from rtos timer interrupt timeslice

bool timerCallBack(void) {
    lv_tick_inc(10);
    return true;
}

// GUI Task
// Could be the same as main task, but need to control affinity, priority, etc.

void guiProcess(void* p) {
//  ILI9341 lcd(); // RESET 18 SCL 19 DC 21 CS 22 SDA 23 SDO 25 Backlight 5
    SSD1306 oled(GPIO_NUM_21, GPIO_NUM_22);

    // Rotary encoder for input
    encoder = new Encoder(GPIO_NUM_26, GPIO_NUM_27);
    button = new Button(GPIO_NUM_34);
    lv_indev_drv_t encoder;
    lv_indev_drv_init(&encoder);
    encoder.type = LV_INDEV_TYPE_ENCODER;
    encoder.read_cb = guiEncoderRead;
    encoder.feedback_cb = NULL;
    ky040_device = lv_indev_drv_register(&encoder);

    // Screen layout
    statusScreen();

    // LVGL tick timer, runs every RTOS tick (10mS) at high priority
    esp_register_freertos_idle_hook_for_cpu(timerCallBack, guiCpuCore);

    // Main loop, executes forever based on hint returned from lv_task_handler()
    while (true) { vTaskDelay(lv_task_handler() / portTICK_PERIOD_MS); }

    // We should never reach this point
    vTaskDelete(NULL);
}
