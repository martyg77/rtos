esp-idf + PlatformIO + LVGL is tricky to get working together

PlaformIO does not currently support Kconfig files imported through its library system
Workaround is to copy the data into main/Kconfig
ref. https://github.com/platformio/platform-espressif32/issues/453
Note tweaking of lvgl.h and others still required when importing though PlatformIO library

Adding as components gets around Kconfig issue, also provides better version control through git

LVGL is still under active development
Finding the right combination of branches that work on esp-idf has been a chore
The https://github.com/lvgl/lv_port_esp32 repo points to a working set of commits to start with

When installing lvgl to components, tweak its CMakeLists.txt and remove reference to "main"
Make sure to do a 'pio run -t menuconfig' before attmepting any compiles

Component/submodule structure cloned from https://github.com/lvgl/lv_port_esp32 @ 538dbbd
lvgl @ b55ee6a -> release/v7
lvgl_esp32_drivers @ 9fed1cc -> master
lv_examples/lv_examples @ 3f41b7d -> release/v7

running on ESP-WROVER-KIT-VE with lcd ili9341 (spi 320x240)
