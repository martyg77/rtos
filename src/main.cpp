#include "Flasher.h"
#include "Motor.h"
#include "Encoder.h"
#include "Segway.h"

#include <driver/gpio.h>
#include <driver/timer.h>
#include <freertos/task.h>
#include <MPU6050.h>

// Motor controller STBY pin; software always enables
// TODO STBY probably should be strapped in hardware

const gpio_num_t gpio_stby = GPIO_NUM_12; // TODO JTAG conflict

void setup_stby_gpio() {
    gpio_pad_select_gpio(gpio_stby);
    gpio_config_t g = {
        .pin_bit_mask = (1ull << gpio_stby),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&g);

    gpio_set_level(gpio_stby, 1);
}

// I2C master port

const i2c_port_t mpu_i2c_port = I2C_NUM_0;
const gpio_num_t mpu_sda = GPIO_NUM_21;
const gpio_num_t mpu_scl = GPIO_NUM_22;

void i2c_setup() {
    i2c_config_t c = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = mpu_sda,
        .scl_io_num = mpu_scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = 400000}, // MPU6850 fSCL max 400kHz
    };
    i2c_param_config(mpu_i2c_port, &c);
    i2c_driver_install(mpu_i2c_port, I2C_MODE_MASTER, 0, 0, 0);
}

// 5mS timebase for Segway controller task

const timer_group_t group = TIMER_GROUP_0;
const timer_idx_t timer = TIMER_0;

const int clock_prescaler = 16;
const int clock_frequency = TIMER_BASE_CLK / clock_prescaler; // Base clock APB 80MHz 
const uint64_t timer5mS_preset = clock_frequency * Segway::handlerIntervalmS / 1000;

bool timer5mS_ISR(Segway *robot) {
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(robot->task, &wake);
    if (wake == pdTRUE) portYIELD_FROM_ISR(); // Request context switch
    return wake == pdTRUE;
}

void timer5mS_enable(Segway *robot) {
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_DOWN,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = clock_prescaler,
    };
    timer_init(group, timer, &config);

    timer_set_counter_value(group, timer, timer5mS_preset);
    timer_isr_callback_add(group, timer, (timer_isr_t)timer5mS_ISR, robot, 0);
    timer_start(group, timer);
}

// Main program

extern "C" { void app_main(); }

void app_main() {
    gpio_install_isr_service(0);
    i2c_setup();
    setup_stby_gpio(); // TODO JTAG conflict

    Flasher blue(GPIO_NUM_2, 250);

    Encoder right_encoder(GPIO_NUM_17, GPIO_NUM_16);
    Encoder left_encoder(GPIO_NUM_26, GPIO_NUM_25);

    Motor left_motor(GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_13, LEDC_TIMER_0, LEDC_CHANNEL_0); 
    // Motor left_motor(GPIO_NUM_33, GPIO_NUM_27, GPIO_NUM_32, LEDC_TIMER_0, LEDC_CHANNEL_0); // JTAG conflicts
    Motor right_motor(GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_5, LEDC_TIMER_1, LEDC_CHANNEL_1);

    MPU6050 mpu;
    mpu.initialize(); // TODO Where does MPU6050 _calibration_ take place?

    Segway robot(&left_motor, &right_motor, &left_encoder, &right_encoder, &mpu);

    vTaskDelay(2500 / portTICK_PERIOD_MS); // Allow time for robot to stablize after power-on
    timer5mS_enable(&robot);

    while (false) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("%i %i %i %i %i %i %f\n",
               robot.accelX, robot.accelY, robot.accelZ, robot.gyroX, robot.gyroY, robot.gyroZ, robot.kalmanfilter.angle);
    }

    const int speed = 50;
    while (false) {
        printf("Encoders %i %i\n", left_encoder.value(), right_encoder.value());
        left_motor.run(speed);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Encoders %i %i\n", left_encoder.value(), right_encoder.value());
        left_motor.run(-speed);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        left_motor.stop();
        printf("Encoders %i %i\n", left_encoder.value(), right_encoder.value());
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        printf("Encoders %i %i\n", left_encoder.value(), right_encoder.value());
        right_motor.run(speed);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Encoders %i %i\n", left_encoder.value(), right_encoder.value());
        right_motor.run(-speed);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        right_motor.stop();
        printf("Encoders %i %i\n", left_encoder.value(), right_encoder.value());
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }

    // This procedure must never return
    while (true) vTaskDelay(portMAX_DELAY);
}
