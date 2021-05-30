#include "Motor.h"

#include <driver/ledc.h>

Motor::Motor(const gpio_num_t in1, const gpio_num_t in2, const gpio_num_t pwm,
      const ledc_timer_t timer, const ledc_channel_t channel) {
    gpio_IN1 = in1;
    gpio_IN2 = in2;
    gpio_PWM = pwm;
    pwm_timer = timer;
    pwm_channel = channel;

    gpio_pad_select_gpio(gpio_IN1);
    gpio_pad_select_gpio(gpio_IN2);
    gpio_config_t g = {
        .pin_bit_mask = (1ull << gpio_IN1) | (1ull << gpio_IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&g);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, // TODO does high-speed mode buy me anything?
        {.duty_resolution = LEDC_TIMER_8_BIT}, // TODO PWM resolution +/- 256 preferred
        .timer_num = pwm_timer,
        .freq_hz = 5000, // TODO controller max 100kHz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel = {
        .gpio_num = gpio_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = pwm_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = pwm_timer,
        .duty = 0,
        .hpoint = 0, // TODO not sure what this does
    };
    ledc_channel_config(&ledc_channel);

    stop();
}

void Motor::run(const int duty_cycle) {
    assert(abs(duty_cycle) <= duty_cycle_range);

    if (duty_cycle > 0) { // forward direction
        gpio_set_level(gpio_IN1, 1);
        gpio_set_level(gpio_IN2, 0);
    } else { // reverse direction
        gpio_set_level(gpio_IN1, 0);
        gpio_set_level(gpio_IN2, 1);
    }

    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, abs(duty_cycle));
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

void Motor::stop() {
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 0);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    
    gpio_set_level(gpio_IN1, 0);
    gpio_set_level(gpio_IN2, 0);
}
