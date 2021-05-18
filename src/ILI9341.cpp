#include "ILI9341.h"

#include <assert.h>
#include <driver/i2c.h>
#include <lv_font/lv_font.h>
#include <lv_themes/lv_theme.h>
#include <lv_themes/lv_theme_mono.h>
#include <lvgl_helpers.h>
#include <lvgl_tft/ili9341.h>

ILI9341::ILI9341() {
    // Allocate DMA-capable memory for display double-buffering
    buf1 = (lv_color_t *)heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    buf2 = (lv_color_t *)heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);

    // Initialize LVGL susbsystem
    lv_disp_buf_t buffer;
    lv_disp_buf_init(&buffer, buf1, buf2, DISP_BUF_SIZE);
    lv_disp_drv_t display;
    lv_disp_drv_init(&display);
    display.buffer = &buffer;
    display.flush_cb = disp_driver_flush;
    lv_disp_drv_register(&display);

    // Initialize driver hardware
    lvgl_driver_init();
}

ILI9341::~ILI9341() {
    free(buf1);
    free(buf2);
    // TODO unregister display
}
