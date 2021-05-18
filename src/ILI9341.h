// LVGL ili9341 display driver
// C++ class shim for LVGL supplied display driver

// For simplicity, implementation is hard-coded to wrover-kit SBC display
// The subtending driver is currently configured through Kconfig
// RESET 18 SCL 19 DC 21 CS 22 SDA 23 SDO 25 Backlight 5

#ifndef ILI9341_H
#define ILI9341_H

#include <driver/gpio.h>
#include <lvgl.h>

class ILI9341 {
  public:
    ILI9341();
    ~ILI9341();

  private:
    lv_color_t *buf1 = nullptr;
    lv_color_t *buf2 = nullptr;
    lv_disp_drv_t display;
};

#endif // ILI9341_H