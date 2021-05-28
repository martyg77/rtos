// AsLong JGB37-520B-333RPM

// Toshiba TB6612FNG 
// fPWM 100kHz
// TODO add brake functions - do they buy me anything?

// LED (really general purpose) PWM (LEDC) controller
// PWM duty cycle controls motor speed
// Motor supports forward/reverse, specified using signed duty cycle

#pragma once

#include <driver/gpio.h>
#include <hal/ledc_types.h>

class Motor {
  public:
    Motor(const gpio_num_t in1, const gpio_num_t in2, const gpio_num_t pwm,
            const ledc_timer_t timer, const ledc_channel_t channel);

    // TODO duty cycle type is really +/- 255, not int
    // TODO express motor PWM as float 0 <= x <= 1uty cycle
    const uint duty_cycle_range = 255; // Maximum value for PWM d
    void run(const int duty_cycle); // negative == reverse
    void stop();

  private:
    gpio_num_t gpio_IN1 = GPIO_NUM_NC;
    gpio_num_t gpio_IN2 = GPIO_NUM_NC;
    gpio_num_t gpio_PWM = GPIO_NUM_NC;

    ledc_timer_t pwm_timer = LEDC_TIMER_MAX;
    ledc_channel_t pwm_channel = LEDC_CHANNEL_MAX;

    ledc_timer_config_t ledc_timer;
    ledc_channel_config_t ledc_channel;
};
