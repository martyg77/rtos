// LVGL ssd1306 display driver
// Ref. https://github.com/nopnop2002/esp-idf-ssd1306 for hardware definitions and init sequence
// Ref. https://github.com/espressif/esp-idf/tree/master/examples/peripherals/spi_master/lcd
// Ref. LVGL monochrome configuration https://blog.lvgl.io/2019-05-06/oled
// Ref. Solomon Systech ssd1306 datasheet, Rev. 1.1

// For simplicity, implementation is hard-wired to 128x64 i2c ssd1306 display
// This is a monochrome device; top 16 pixel rows have yellow phosphor, remaining 48 are blue

#include <driver/gpio.h>
#include <driver/i2c.h>
#include "lvgl.h"

class SSD1306 {
public:
    SSD1306(gpio_num_t sda, gpio_num_t scl);
    ~SSD1306();
    
private:
    gpio_num_t sda = GPIO_NUM_NC;
    gpio_num_t scl = GPIO_NUM_NC;

    // TODO several of these should be constructor arguments
    static const i2c_port_t port = 0;
    static const int i2cAddress = 0x3c;
    static const int width = 128; // pixels
    static const int height = 64; // pixels
    static const int displayBufferSize = width * height / 8; // bytes

    lv_disp_drv_t display;
    lv_disp_buf_t buffer;
    uint8_t *displayBuffer = NULL;

    static void i2cDispatch(uint8_t prefix, uint8_t *msg, int len);
    
    // LVGL driver callbacks
    static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
    static void rounder_cb(lv_disp_drv_t *drv, lv_area_t *area);
    static void set_px_cb(lv_disp_drv_t *drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                          lv_color_t color, lv_opa_t opa);
};
