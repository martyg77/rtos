#include "ESP32Encoder.h"

// This handler triggered whenever the 16-bit PCNT hardware counter overflows
// TODO future API pcnt_get_event_status() will greatly simplify this handler

ESP32Encoder *ESP32Encoder::encoders[PCNT_UNIT_MAX];

static void IRAM_ATTR overflowHandler(void *p) {
    for (int u = 0; u < PCNT_UNIT_MAX; ++u) 
        if (PCNT.int_st.val & (BIT(u))) {
            ESP32Encoder *ptr = ESP32Encoder::encoders[u];

            if (PCNT.status_unit[u].h_lim_lat)
                ptr->tally += ESP32Encoder::counterMAX;
            else if (PCNT.status_unit[u].l_lim_lat)
                ptr->tally -= ESP32Encoder::counterMAX;

            PCNT.int_clr.val = BIT(u); // Clear the interrupt for this unit
        }
}

void esp32_encoder_install() {
    for (int u = 0; u < PCNT_UNIT_MAX; ++u) ESP32Encoder::encoders[u] = nullptr;
    pcnt_isr_register(overflowHandler, nullptr, 0, nullptr);
}

ESP32Encoder::ESP32Encoder(gpio_num_t a, gpio_num_t b, pcnt_unit_t u) {
    pinA = a;
    pinB = b;
    unit = u;
    encoders[unit] = this;

    // PCNT channel 0
    pcnt_config_t p;
    p.pulse_gpio_num = pinA; // input
    p.ctrl_gpio_num = pinB; //gate
    p.lctrl_mode = PCNT_MODE_REVERSE;
    p.hctrl_mode = PCNT_MODE_KEEP;
    p.pos_mode = PCNT_COUNT_DEC;
    p.neg_mode = PCNT_COUNT_INC;
    p.counter_h_lim = counterMAX; // INT16_MAX;
    p.counter_l_lim = -counterMAX; // INT16_MIN;
    p.unit = unit;
    p.channel = PCNT_CHANNEL_0;
    pcnt_unit_config(&p);

    // PCNT channel 1, note signal pins and control modes mirror channel 0
    p.pulse_gpio_num = pinB;
    p.ctrl_gpio_num = pinA;
    p.pos_mode = PCNT_COUNT_INC;
    p.neg_mode = PCNT_COUNT_DEC;
    p.channel = PCNT_CHANNEL_1;
    pcnt_unit_config(&p);

    // Filter out bounces and noise (runt pulses)
    setFilter(250); 

    pcnt_event_enable(unit, PCNT_EVT_H_LIM); // Overflow
    pcnt_event_enable(unit, PCNT_EVT_L_LIM); // Underflow

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_intr_enable(unit);
    pcnt_counter_resume(unit);
}

void ESP32Encoder::clearCount() {
    tally = 0;
    pcnt_counter_clear(unit);
}

// Peek counter value directly from hardware
int16_t ESP32Encoder::getHWCount() { 
    int16_t c;
    pcnt_get_counter_value(unit, &c);
    return c;
}

void ESP32Encoder::setFilter(uint16_t clk) {
    assert(clk < 1024); // APB_CLK pulses, 10-bit register
    pcnt_set_filter_value(unit, clk);
    clk ? pcnt_filter_enable(unit) : pcnt_filter_disable(unit);
}

int ESP32Encoder::delta() {
    int64_t c = getCount();
    int d = c - tallyPrev;
    tallyPrev = c;
    return d;
};
