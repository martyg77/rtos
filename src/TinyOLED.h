// LVGL display wrapper to ssd1306 driver component https://github.com/nopnop2002/esp-idf-ssd1306
// Ref. https://github.com/espressif/esp-idf/tree/master/examples/peripherals/spi_master/lcd

// For simplicity, implementation is hard-wired to 128x64 i2c ssd1306 display
// Implementation uses Page Addressing Mode on target device
// This is a monochrome display; top 16 pixel rows have yellow phosphor, remaining 48 are blue

#include <driver/gpio.h>
#include <driver/i2c.h>
#include "lvgl.h"

class TinyOLED {
public:
    TinyOLED(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
    ~TinyOLED();

    void demo(); // TODO Testing remove
    
    // LVGL driver callbacks

    static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);

    static void rounder_cb(lv_disp_drv_t *drv, lv_area_t *area);

    static void set_px_cb(lv_disp_drv_t *drv, uint8_t *buf, lv_coord_t buf_w,
                           lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t opa);

private:
    i2c_port_t port = 0;
    gpio_num_t sda = GPIO_NUM_NC;
    gpio_num_t scl = GPIO_NUM_NC;

    const int i2cAddress = 0x3c;
    
    // Pixels
    const int width = 128;  // pixels
    const int height = 64;  // pixels
    
    // Characters, based on fixed 8x8 pixel font
    const int xFontPixels = 8;
    const int yFontPixels = 8;
    const int segments = width / xFontPixels; // 7 bits, addresses 8 columns of 16 characters
    const int pages = height / yFontPixels; // 3 bits, addresses 8 rows of 16 characters

    void initHardware();
    void displayImage(int page, int seg, uint8_t * images, int width);
    void displayText(int page, const char * text, int text_len);
    void clearLine(int page);
    void clearScreen();
};
