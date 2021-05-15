#include "assert.h"
#include "driver/i2c.h"
#include "MicroOLED.h"
#include "TinyOLED_hw.h"
#include "lv_themes/lv_theme.h"
#include "lv_themes/lv_theme_mono.h"
#include "lv_font/lv_font.h"

void MicroOLED::i2cDispatch(uint8_t prefix, uint8_t *msg, int len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, prefix, true); // Either OLED_CONTROL_BYTE_CMD_STREAM or OLED_CONTROL_BYTE_DATA_STREAM
    i2c_master_write(cmd, msg, len, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

MicroOLED::MicroOLED(gpio_num_t a, gpio_num_t l) {
    sda = a;
    scl = l;

    // Set up the specified GPIO ports for i2c master
    // Per ssd1306 datasheet Tcycle 100nS typ. (10MHz)
    // esp-idf API documents a 1MHz limit
    i2c_config_t c;
    c.mode = I2C_MODE_MASTER;
    c.sda_io_num = sda;
    c.scl_io_num = scl;
    c.sda_pullup_en = GPIO_PULLUP_ENABLE;
    c.scl_pullup_en = GPIO_PULLUP_ENABLE;
    c.master.clk_speed = 1000000;
    i2c_param_config(port, &c);
	i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);

    // ssd1306 init vector
    uint8_t msg[] = { // TODO what are these constants
        OLED_CMD_DISPLAY_OFF,
        OLED_CMD_SET_MUX_RATIO,
        0x3F,                      
        OLED_CMD_SET_DISPLAY_OFFSET,
        0x00,
        OLED_CONTROL_BYTE_DATA_STREAM,
        OLED_CMD_SET_SEGMENT_REMAP,
        OLED_CMD_SET_COM_SCAN_MODE,
        OLED_CMD_SET_DISPLAY_CLK_DIV,	
        0x80,
        OLED_CMD_SET_COM_PIN_MAP,		
        0x12,                             
        OLED_CMD_SET_CONTRAST,		
        0xFF,
        OLED_CMD_DISPLAY_RAM,		
        OLED_CMD_SET_VCOMH_DESELCT,	
        0x40,
        OLED_CMD_SET_MEMORY_ADDR_MODE,
        OLED_CMD_SET_PAGE_ADDR_MODE,
        0x00,
        0x10,
        OLED_CMD_SET_CHARGE_PUMP,	
        0x14,
        OLED_CMD_DEACTIVE_SCROLL,	
        OLED_CMD_DISPLAY_NORMAL,	
        OLED_CMD_DISPLAY_ON
    };
    i2cDispatch(OLED_CONTROL_BYTE_CMD_STREAM, msg, sizeof(msg));

    // Initialize LVGL susbsystem with monochrome theme
    displayBuffer = (uint8_t *) malloc(displayBufferSize);
    lv_disp_buf_init(&buffer, displayBuffer, NULL, displayBufferSize);

    lv_disp_drv_init(&display);
    display.buffer = &buffer;
    display.hor_res = width;
    display.ver_res = height;
    display.flush_cb = flush_cb;
    display.rounder_cb = rounder_cb;
    display.set_px_cb = set_px_cb;
    lv_disp_drv_register(&display);

    lv_theme_set_act(
        lv_theme_mono_init(LV_COLOR_BLACK, LV_COLOR_WHITE, LV_THEME_DEFAULT_FLAG, NULL,
                           &lv_font_unscii_8, &lv_font_unscii_8, &lv_font_unscii_8));
}

MicroOLED::~MicroOLED() {
    free(displayBuffer);
    // TODO unregister display and i2c port
}

// Bitbang pixels into format conformant to hardware layout, ref. ssd1306 datasheet Fig. 10-2
// Supplying this callback implements a custom-packed displayBuffer, essentially redefining lv_color_t 

void MicroOLED::set_px_cb(lv_disp_drv_t *drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                          lv_color_t color, lv_opa_t opa) 
{
    uint16_t byte = (y >> 3) * buf_w + x;
    uint8_t bit = y & 0x7;

    #define BIT_SET(a,b) ((a) |= (1U<<(b)))
    #define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))

    if (color.full)
        BIT_SET(buf[byte], bit);
    else
        BIT_CLEAR(buf[byte], bit);
}

// Adjust supplied display area so it aligns to displayBuffer byte boundaries

void MicroOLED::rounder_cb(lv_disp_drv_t *drv, lv_area_t *area) {
    area->y1 = area->y1 & ~0x7;
    area->y2 = area->y2 | 0x7;
}
    
// Transmit rectangular area of displayBuffer to ssd1306 hardware
// LVGL passes back the structure we filled in set_px_cb so np futher reformatting is necessary

void MicroOLED::flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *buf) {
    uint8_t p1 = area->y1 >> 3;
    uint8_t p2 = area->y2 >> 3;

    uint8_t msg[] = {
        OLED_CMD_SET_MEMORY_ADDR_MODE,
        OLED_CMD_SET_HORI_ADDR_MODE,
        OLED_CMD_SET_COLUMN_RANGE,
        (uint8_t) area->x1,
        (uint8_t) area->x2,
        OLED_CMD_SET_PAGE_RANGE,
        (uint8_t) p1,
        (uint8_t) p2,
    };
    i2cDispatch(OLED_CONTROL_BYTE_CMD_STREAM, msg, sizeof(msg));

    i2cDispatch(OLED_CONTROL_BYTE_DATA_STREAM, (uint8_t *) buf, (p2 - p1 + 1) * (area->x2 - area->x1 + 1)); 

    lv_disp_flush_ready(drv);
}
