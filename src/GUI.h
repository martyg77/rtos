// Glue logic into LVGL graphics susbsystem

#ifndef GUI_H
#define GUI_H

#include <sys/types.h>

// LVGL team recommends pinning the process responsible for rendering
static const uint guiCpuCore = 1;  // 0 = PRO; 1 = APP

// GUI task
extern void guiProcess(void* p);

#endif // GUI_H
