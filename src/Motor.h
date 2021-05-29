// AsLong JGB37-520B-333RPM DC motor, geared 30:1
// Toshiba TB6612FNG motor controller

// PWM duty cycle controls motor speed using ESP32 LEDC controller

#pragma once

#include <driver/gpio.h>
#include <hal/ledc_types.h>

class Motor {
  public:
    Motor(const gpio_num_t in1, const gpio_num_t in2, const gpio_num_t pwm,
            const ledc_timer_t timer, const ledc_channel_t channel);

    // TODO duty cycle type is really +/- 255, not int
    // TODO express motor PWM as float 0.0 <= x <= 1.0
    // Motor supports forward/reverse, specified using signed duty cycle
    const uint duty_cycle_range = 255; // Maximum absolute value for PWM duty_cycle
    void run(const int duty_cycle); // negative == reverse
    void stop();

  private:
    gpio_num_t gpio_IN1 = GPIO_NUM_NC;
    gpio_num_t gpio_IN2 = GPIO_NUM_NC;
    gpio_num_t gpio_PWM = GPIO_NUM_NC;

    ledc_timer_t pwm_timer = LEDC_TIMER_MAX;
    ledc_channel_t pwm_channel = LEDC_CHANNEL_MAX;

    ledc_channel_config_t ledc_channel;
};
