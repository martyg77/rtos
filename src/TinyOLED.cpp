#include <driver/gpio.h>
#include <driver/i2c.h>
#include "lvgl.h"

#include "TinyOLED.h"
#include "TinyOLED_hw.h"
#include "font8x8_basic.h"

void TinyOLED::initHardware() {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2cAddress << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);				// AE
	i2c_master_write_byte(cmd, OLED_CMD_SET_MUX_RATIO, true);			// A8
	i2c_master_write_byte(cmd, 0x3F, true);                             // height == 64
	i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_OFFSET, true);		// D3
	i2c_master_write_byte(cmd, 0x00, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);	// 40
	i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true);		// A1
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true);		// C8
	//i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);		// A6
	i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_CLK_DIV, true);		// D5
	i2c_master_write_byte(cmd, 0x80, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_PIN_MAP, true);			// DA
	i2c_master_write_byte(cmd, 0x12, true);                             // height == 64
	i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true);			// 81
	i2c_master_write_byte(cmd, 0xFF, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_RAM, true);				// A4
	i2c_master_write_byte(cmd, OLED_CMD_SET_VCOMH_DESELCT, true);		// DB
	i2c_master_write_byte(cmd, 0x40, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_MEMORY_ADDR_MODE, true);	// 20
	i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_ADDR_MODE, true);		// 02
	i2c_master_write_byte(cmd, 0x00, true);
	i2c_master_write_byte(cmd, 0x10, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);			// 8D
	i2c_master_write_byte(cmd, 0x14, true);
	i2c_master_write_byte(cmd, OLED_CMD_DEACTIVE_SCROLL, true);			// 2E
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);			// A6
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);				// AF
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(port, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

TinyOLED::TinyOLED(i2c_port_t p, gpio_num_t a, gpio_num_t l)
{
    port = p;
    sda = a;
    scl = l;

    i2c_config_t c;
    c.mode = I2C_MODE_MASTER;
    c.sda_io_num = sda;
    c.scl_io_num = scl;
    c.sda_pullup_en = GPIO_PULLUP_ENABLE;
    c.scl_pullup_en = GPIO_PULLUP_ENABLE;
    c.master.clk_speed = 1000000; // TODO c++ union initializer wonkiness
    i2c_param_config(port, &c);
	i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);

    initHardware();
}

TinyOLED::~TinyOLED() {}

void TinyOLED::flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *map) {
    // copy a buffer's content to a specific area of the display
    lv_disp_flush_is_last(drv);// TODO reday after each call, or the last?
    lv_disp_flush_ready(drv);
}

void TinyOLED::rounder_cb(lv_disp_drv_t *drv, lv_area_t *area) {
    // round the coordinates of areas to redraw (i.e. pad to 8-bit boundaries)
    // Update the areas as needed - Can be only larger
    area->y1 = area->y1 & 0x07;
    area->y2 = (area->y2 & 0x07) + 8;
}

void TinyOLED::set_px_cb(lv_disp_drv_t *drv, uint8_t *buf, lv_coord_t buf_w,
                         lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t opa) {
    // custom function to write the display buffer
    // TODO do I need this?
    // Should only be relevant to large displays, to save buffer memory
}

void TinyOLED::displayImage(int page, int seg, uint8_t * images, int width) {
    assert(page < pages);
//  assert(seg < segments); // TODO Why?

    // Set cursor
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2cAddress << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	uint8_t columLow = seg & 0x0f;
	uint8_t columHigh = (seg >> 4) & 0x0f;
	i2c_master_write_byte(cmd, (0x00 + columLow), true);
	i2c_master_write_byte(cmd, (0x10 + columHigh), true);
	i2c_master_write_byte(cmd, 0xB0 | page, true);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

    // Write images sequentially from above starting point
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (i2cAddress << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
	i2c_master_write(cmd, images, width, true);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(port, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

void TinyOLED::displayText(int page, const char * text, int len) {
	assert(page < pages);
	if (len > 16) len = 16; // At most 16 charcters per line

    const int fontBytes = 8; // font 8x8 - 64 bits/character
    uint8_t image[fontBytes]; // TODO writing one message per character here?
    for (int i = 0, seg = 0; i < len; i++, seg = seg + fontBytes) {
        memcpy(image, font8x8_basic_tr[(uint8_t)text[i]], fontBytes);
        displayImage(page, seg, image, fontBytes);
    }
}

void TinyOLED::clearLine(int page) {
    char space[16]; // TODO Display character or pixel based?
    memset(space, 0x20, sizeof(space));
    displayText(page, space, sizeof(space));
}

void TinyOLED::clearScreen() { for (int i = 0; i < pages; i++) clearLine(i); }

void TinyOLED::demo() {
    clearScreen();
	displayText(0, "SSD1306 128x64", 14);
	displayText(1, "ABCDEFGHIJKLMNOP", 16);
	displayText(2, "abcdefghijklmnop",16);
	displayText(3, "Hello World!!", 13);
    displayText(4, "ssd1306 128x64", 14);
	displayText(5, "Hello World!!", 13);
	displayText(6, "abcdefghijklmnop",16);
	displayText(7, "Hello World!!", 13);
    clearLine(5);
}
