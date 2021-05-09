// Glue logic into LVGL graphics susbsystem

// LVGL team recommends pinning the process responsible for rendering
static const BaseType_t guiCpuCore = 1;  // 0 = PRO; 1 = APP

// GUI Task
extern void guiProcess(void* p);
