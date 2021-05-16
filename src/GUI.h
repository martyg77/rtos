// Glue logic into LVGL graphics susbsystem

#include "lvgl.h"
#include "Button.h"
#include "Encoder.h"

// LVGL team recommends pinning the process responsible for rendering
static const BaseType_t guiCpuCore = 1;  // 0 = PRO; 1 = APP

// GUI task
extern void guiProcess(void* p);
