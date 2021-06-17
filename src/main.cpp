#include "ESP32Encoder.h"
#include "Flasher.h"
#include "Motor.h"
#include "Segway.h"
#include "WiFi.h"

#include <MPU6050.h>
#include <assert.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <driver/timer.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <nvs_flash.h>

// Ensure WiFi credentials don't get uploaded to GitHub
// #define WIFI_SSID        "myssid"
// #define WIFI_PASSWORD    "mypassword"
#include ".wifi"

// Motor controller STBY pin; software always enables
// TODO strap STBY high in hardware, save the GPIO pin

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

void network_state_cb(WiFi *p) { if (!p->online) p->reconnect(); }

extern "C" { void app_main(); }

void app_main() {
    gpio_install_isr_service(0);
    esp32_encoder_install();
    i2c_setup();

    assert(nvs_flash_init() == ESP_OK);
    esp_netif_init();
    esp_event_loop_create_default();
    esp_wifi_set_default_wifi_sta_handlers();

#ifndef JTAG
    setup_stby_gpio();
#endif

    Flasher red(GPIO_NUM_2, 250);

    ESP32Encoder right_encoder(GPIO_NUM_17, GPIO_NUM_16, PCNT_UNIT_0);
    ESP32Encoder left_encoder(GPIO_NUM_26, GPIO_NUM_25, PCNT_UNIT_3);

#ifdef JTAG
    Motor left_motor(GPIO_NUM_33, GPIO_NUM_27, GPIO_NUM_32, LEDC_TIMER_0, LEDC_CHANNEL_0);
#else
    Motor left_motor(GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_13, LEDC_TIMER_0, LEDC_CHANNEL_0); 
#endif
    Motor right_motor(GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_5, LEDC_TIMER_1, LEDC_CHANNEL_1);

    // Motion processor: SDA = GPIO_NUM_21; SCL = GPIO_NUM_22
    MPU6050 mpu;
    mpu.initialize(); // TODO Where does MPU6050 _calibration_ take place?

    robot = new Segway(&left_motor, &right_motor, &left_encoder, &right_encoder, &mpu);

    WiFi network;
    network.state_cb = network_state_cb; 
    network.connect(WIFI_SSID, WIFI_PASSWORD);
    while (!network.online) vTaskDelay(100 / portTICK_PERIOD_MS);

    // TODO move to Segway constructor?
    Echo echo(3333);
    Telemetry telemetry(4444);
    Helm cockpit(5555);
    Console console(6666);

    vTaskDelay(2500 / portTICK_PERIOD_MS); // Allow time for robot to stablize after power-on
    timer5mS_enable(robot);

    // This procedure must never return
    while (true) vTaskDelay(portMAX_DELAY);
}
