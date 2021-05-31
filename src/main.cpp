#include "Flasher.h"
#include "Motor.h"
#include "Encoder.h"
#include "Segway.h"

#include <driver/gpio.h>
#include <driver/timer.h>
#include <MPU6050.h>

// Motor controller STBY pin; software always enables
// TODO STBY probably should be strapped in hardware

const gpio_num_t gpio_stby = GPIO_NUM_12;

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
const gpio_num_t mpu_sda = GPIO_NUM_22;
const gpio_num_t mpu_scl = GPIO_NUM_21;

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

// Base component initialization required *BEFORE* other global constuctors
class Init {
  public:
    Init() {
        gpio_install_isr_service(0);
        vTaskDelay(50);
        // lv_init();
        // setup_stby_gpio(); // JTAG conflict
        i2c_setup();
    }
} init_me_first;

const gpio_num_t GPIO_LED_RED = GPIO_NUM_0;
const gpio_num_t GPIO_LED_GREEN = GPIO_NUM_2;
const gpio_num_t GPIO_LED_BLUE = GPIO_NUM_4;

// Flasher red(GPIO_LED_RED, 250);
Flasher green(GPIO_LED_GREEN, 300);
// Flasher blue(GPIO_LED_BLUE, 500);

Encoder left_encoder(GPIO_NUM_26, GPIO_NUM_25);
Encoder right_encoder(GPIO_NUM_17, GPIO_NUM_16);

//Motor left_motor(GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_13, LEDC_TIMER_0, LEDC_CHANNEL_0); // JTAG conflicts
  Motor left_motor(GPIO_NUM_33, GPIO_NUM_27, GPIO_NUM_32, LEDC_TIMER_0, LEDC_CHANNEL_0);

Motor right_motor(GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_5, LEDC_TIMER_1, LEDC_CHANNEL_1);

MPU6050 mpu;

// So now I need to control the thing, but just getting it to stand straight is enough for now
Segway robot(&left_motor, &right_motor, &left_encoder, &right_encoder, &mpu);

// 5mS timebase

const timer_group_t group = TIMER_GROUP_0;
const timer_idx_t timer = TIMER_0;

const int clock_prescaler = 16;
const int clock_frequency = TIMER_BASE_CLK / clock_prescaler; // Base clock APB 80MHz 
const uint64_t timer5mS_preset = clock_frequency * Segway::handlerIntervalmS / 1000;

bool timer5mS_callback(void *p) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; // TODO not sure what this does
    xEventGroupSetBitsFromISR(robot.event, 1, &xHigherPriorityTaskWoken);
    return false; // TODO true to force reschedule?
}

void timer_setup() {
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
    timer_isr_callback_add(group, timer, timer5mS_callback, nullptr, 0);
    timer_start(group, timer);
}

// Main program

extern "C" { void app_main(); }

void app_main() {
    mpu.initialize(); // TODO Where does MPU6050 calibration take place?
    timer_setup();

    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /*
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        printf("%i %i %i %i %i %i\n", ax, ay, az, gx, gy, gz);
        */
        printf("%i %i %i %i %i %i %f\n", 
            robot.accelX, robot.accelY, robot.accelZ, robot.gyroX, robot.gyroY, robot.gyroZ, robot.kalmanfilter.angle);
    }

    while (true) {
        left_motor.run(50);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        left_motor.run(-50);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        left_motor.stop();
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        right_motor.run(50);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        right_motor.run(-50);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        right_motor.stop();
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}
