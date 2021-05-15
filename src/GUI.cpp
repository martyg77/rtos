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

// LVGL 10mS tick timebase, called directly from rtos timer interrupt

bool timerCallBack(void) {
    lv_tick_inc(10);
    return true;
}

void demo()
{
    lv_obj_t *scr = lv_obj_create(NULL, NULL);
    lv_scr_load(scr);                                /*Load the screen*/
    lv_obj_t *btn1 = lv_btn_create(scr, NULL);      /*Create a button on the screen*/
    lv_btn_set_fit(btn1, true);                      /*Enable to automatically set the size according to the content*/
    lv_obj_set_pos(btn1, 20, 10);                    /*Set the position of the button*/
    lv_obj_t *btn2 = lv_btn_create(scr, btn1);      /*Copy the first button*/
    lv_obj_set_pos(btn2, 40, 20);                    /*Set the position of the button*/
    lv_obj_t *label1 = lv_label_create(btn1, NULL); /*Create a label on the first button*/
    lv_label_set_text(label1, "Button 1");           /*Set the text of the label*/
    lv_obj_t *label2 = lv_label_create(btn2, NULL); /*Create a label on the second button*/
    lv_label_set_text(label2, "Button 2");           /*Set the text of the label*/
    lv_obj_del(label1);
}

// GUI Task

void guiProcess(void* p) {
 
    // Initialize LVGL susbsystem
    MicroOLED oled(GPIO_NUM_21, GPIO_NUM_22);
    Button ky040_button(GPIO_NUM_25);
    Encoder ky040_encoder(GPIO_NUM_26, GPIO_NUM_27);
    demo();

    // LVGL tick timer, runs every RTOS tick (10mS) at high priority
    esp_register_freertos_idle_hook_for_cpu(timerCallBack, guiCpuCore);

    // Main loop, executes forever based on hint returned from lv_task_handler()
    while (true) { vTaskDelay(lv_task_handler() / portTICK_PERIOD_MS); }

    // We should never reach this point
    vTaskDelete(NULL);
}
