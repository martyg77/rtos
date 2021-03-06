#include "Segway.h"

#include <driver/timer.h>
#include <esp_console.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <linenoise/linenoise.h>
#include <string.h>

// TODO Arduino artifact goodies, where is esp-idf/freertos equivalent
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define min(x, y) ((x) < (y) ? (x) : (y))
#define UNUSED(x) (void)(x) // Ref. https://stackoverflow.com/questions/3599160

// Segway class is a singleton
Segway *robot = nullptr;

// Tilt (vertical balancing) feedback loop
// Yields motor PWM requied for vertical balance
// This code runs every 5mS

float Segway::tiltPID() {
    const float mpu_gyro_scaling = 131; // internal unit -> degree (FS_SEL=0)
    const float mpu_accel_scaling = 16384; // internal unit -> g (AFS_SEL= 0)
    UNUSED(mpu_accel_scaling); // Suppress compiler warning
    const float radians2degrees = 180 / M_PI;

    // Retrieve raw 6-axis data from inertial sensor, normalize gyro angle to degrees
    // Ref. https://mjwhite8119.github.io/Robots/mpu6050
    int16_t ax, ay, az; // raw 3-axis accelerometer (linear acceleration, internal unit)
    int16_t gx, gy, gz; // raw 3-axis gyroscope (angular acceleration, internal unit)
    mpu->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    GyroX = gx / mpu_gyro_scaling; // Angular velocity about yz-plane (pitch)
    GyroY = gy / mpu_gyro_scaling; // Angular velocity about xz-plane (roll)
    GyroZ = gz / mpu_gyro_scaling; // Angular velocity about xy-plane (yaw)

    // Calculate (noise filtered) tilt angle from mpu raw data
    // Complementary filter - low pass accelerometer, high pass gyroscope
    tiltAngle = kalman.filter(atan2(ay, az) * radians2degrees, GyroX);
    tiltAcceleration = kalman.angle_accel;

    // PID calculation
    return tiltControl = tilt.Kp * (tiltSetPoint - tiltAngle) + tilt.Kd * -tiltAcceleration;
}

// Linear (forward/back) velocity feedback loop
// Yields motor PWM required for requested movement
// Indirectly controls station keeping while at rest
// This code runs every 50mS

float Segway::speedPID() {

    // Estimate overall speed since last invocation, i.e. (d1-d0)/50mS
    int delta = min(left_encoder->delta(), right_encoder->delta());

    // Filter speed samples through low-pass filter to yield indicated speed
    // Ref. https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
    // Stated purpose is noise reduction for above samples, control interference with (of?) tilt feedback loop
    // TODO I don't think this filter is necessary/ with my hardware
    float s = speedOMeter * 0.7 + delta * 0.3;
    speedOMeter = s;

    // PID calculation
    const float error = speedSetPoint - speedOMeter;
    speedErrorIntegral += error;
    return speedControl = speed.Kp * error + speed.Ki * -speedErrorIntegral;
}

// Angular (turn/spin) velocity feedback loop
// Yields rotational difference between two motors
// This code runs every 20mS

float Segway::turnPID() {

    // TODO GyroZ unfiltered noisy data
    float turnError = turnSetPoint - 0; // TODO need filtered yaw measurement

    // PID calculation
    return turnControl = turn.Kp * turnError + turn.Kd * -GyroZ;
}

// Superpose tilt/turn/speed control outputs and set motor speeds
// TODO increase PWM resolution to 1024 (all in hardware, should cost nothing)
// Note motors are mounted opposite each other, but wired backwards, so they move in same direction
// This code runs every 5mS

void Segway::setPWM() {

    // Speed and turn terms are interference for tilt
    leftMotorPWM = (int)(-tiltControl - speedControl - turnControl);
    rightMotorPWM = (int)(-tiltControl - speedControl + turnControl);
    // leftMotorPWM = constrain(leftMotorPWM, -left_motor->duty_cycle_range, left_motor->duty_cycle_range);
    // rightMotorPWM = constrain(rightMotorPWM, -right_motor->duty_cycle_range, right_motor->duty_cycle_range);
    leftMotorPWM = constrain(leftMotorPWM, -255, 255);
    rightMotorPWM = constrain(rightMotorPWM, -255, 255);

    // If the robot is about to fall over, or already lying on its side, stop both motors
    if (abs(tiltAngle) > 30) leftMotorPWM = rightMotorPWM = 0;

    // Set speed and direction on both motors
    if (!motorEnable) return;
    left_motor->run(leftMotorPWM);
    right_motor->run(rightMotorPWM);
}

// Task mainline

void segwayProcess(Segway *robot) {
    while (true) {

        // Notification raised by timer interrupt every 5mS
        ulTaskNotifyTake(true, portMAX_DELAY);

        // These methods oversee real-time motion control for the robot
        const int every20mS = 20 / robot->handlerIntervalmS; // Used to schedule turn PID
        const int every50mS = 50 / robot->handlerIntervalmS; // Used to schedule speed PID
        robot->tick = (robot->tick + 1) % (every20mS * every50mS); // Prevent integer overflow
        robot->tiltPID();
        // if (!(robot->tick % every50mS)) robot->speedPID();
        // if (!(robot->tick % every20mS)) robot->turnPID();
        robot->setPWM();
    }
}

void Segway::stop() {
    speedSetPoint = 0;
    // TODO turn needs to be aborted here
    left_motor->stop();
    right_motor->stop();
}

Segway::Segway(Motor *lm, Motor *rm, ESP32Encoder *le, ESP32Encoder *re, MPU6050 *m) {
    left_motor = lm;
    right_motor = rm;
    left_encoder = le;
    right_encoder = re;
    mpu = m;

    stop();

    xTaskCreate((TaskFunction_t)segwayProcess, "segway", configMINIMAL_STACK_SIZE * 4, this, 6, &task);
}

// Helm interface

void Helm::service() {
    int c;

    // Design intent is to use numeric keypad as makeshift joystick
    while ((c = fgetc(stdin)) != EOF) {
        switch (c) {
        case '2':
            robot->speedSetPoint -= 25;
            break;
        case '4':
            robot->turnSetPoint = (robot->turnSetPoint - 30) % 360;
            break;
        case '6':
            robot->turnSetPoint = (robot->turnSetPoint + 30) % 360;
            break;
        case '8':
            robot->speedSetPoint += 25;
            break;
        default:
            robot->stop();
            break;
        }
    }
}

// Telemetry interface

void Telemetry::service() {
    while (true) {
        int x = printf("%.2f %.2f %.2f\n", robot->tiltAngle, robot->tiltAcceleration, robot->tiltControl);
        if (x < 0) return;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// Console interface

static struct {
    struct arg_dbl *Kp = arg_dbl1(nullptr, nullptr, "<Kp>", "PID proportional gain");
    struct arg_dbl *Ki = arg_dbl1(nullptr, nullptr, "<Ki>", "PID integral gain");
    struct arg_dbl *Kd = arg_dbl1(nullptr, nullptr, "<Kd>", "PID differential gain");
    struct arg_end *end = arg_end(5);
} tilt_args;

int Console::tilt(int argc, char **argv) {
    if (arg_parse(argc, argv, (void **)&tilt_args) > 0) {
        arg_print_errors(stdout, tilt_args.end, argv[0]);
        return 1;
    }

    robot->tilt.Kp = tilt_args.Kp->dval[0];
    robot->tilt.Ki = tilt_args.Ki->dval[0];
    robot->tilt.Kd = tilt_args.Kd->dval[0];
    return 0;
}

static struct {
    struct arg_dbl *Kp = arg_dbl1(nullptr, nullptr, "<Kp>", "PID proportional gain");
    struct arg_dbl *Ki = arg_dbl1(nullptr, nullptr, "<Ki>", "PID integral gain");
    struct arg_dbl *Kd = arg_dbl1(nullptr, nullptr, "<Kd>", "PID differential gain");
    struct arg_end *end = arg_end(5);
} speed_args;

int Console::speed(int argc, char **argv) {
    if (arg_parse(argc, argv, (void **)&speed_args) > 0) {
        arg_print_errors(stdout, speed_args.end, argv[0]);
        return 1;
    }

    robot->speed.Kp = speed_args.Kp->dval[0];
    robot->speed.Ki = speed_args.Ki->dval[0];
    robot->speed.Kd = speed_args.Kd->dval[0];
    return 0;
}

static struct {
    struct arg_dbl *Kp = arg_dbl1(nullptr, nullptr, "<Kp>", "PID proportional gain");
    struct arg_dbl *Ki = arg_dbl1(nullptr, nullptr, "<Ki>", "PID integral gain");
    struct arg_dbl *Kd = arg_dbl1(nullptr, nullptr, "<Kd>", "PID differential gain");
    struct arg_end *end = arg_end(5);
} turn_args;

int Console::turn(int argc, char **argv) {
    if (arg_parse(argc, argv, (void **)&turn_args) > 0) {
        arg_print_errors(stdout, turn_args.end, argv[0]);
        return 1;
    }

    robot->turn.Kp = turn_args.Kp->dval[0];
    robot->turn.Ki = turn_args.Ki->dval[0];
    robot->turn.Kd = turn_args.Kd->dval[0];
    return 0;
}

static struct {
    struct arg_end *end = arg_end(5);
} pid_args;

int Console::pid(int argc, char **argv) {
    printf("Tilt: Kp %.2f Ki %.2f Kd %.2f\n", robot->tilt.Kp, robot->tilt.Ki, robot->tilt.Kd);
    printf("Speed: Kp %.2f Ki %.2f Kd %.2f\n", robot->speed.Kp, robot->speed.Ki, robot->speed.Kd);
    printf("Turn: Kp %.2f Ki %.2f Kd %.2f\n", robot->turn.Kp, robot->turn.Ki, robot->turn.Kd);
    return 0;
}

static struct {
    struct arg_int *enable = arg_int1(nullptr, nullptr, "<enable>", "");
    struct arg_end *end = arg_end(5);
} motor_args;

int Console::motor(int argc, char **argv) {
    robot->motorEnable = motor_args.enable->ival[0];
    return 0;
}

void Console::service() {
    char s[256];
    int rc;

    while (fgets(s, sizeof(s), stdin)) {
        strtok(s, "\n"); // Strip trailing newline
        strtok(s, "\r"); // Strip trailing carriage return

        switch (esp_err_t err = esp_console_run(s, &rc)) {
        case ESP_ERR_NOT_FOUND:
            printf("Unrecognized command: %s\n", s);
            break;
        case ESP_ERR_INVALID_ARG:
            break; // Command was empty
        case ESP_OK:
            if (rc != ESP_OK)
                printf("Command returned non-zero error code: 0x%x (%s)\n", rc, esp_err_to_name(rc));
            break;
        default:
            printf("Internal error: %s\n", esp_err_to_name(err));
            break;
        }
        fflush(stdout);
    }
}

void Console::parser() {
    const char *prompt = "> ";
    int rc;

    while (true) {
        char *s = linenoise(prompt);
        if (!s) break;
        if (strlen(s) > 0) linenoiseHistoryAdd(s);

        switch (esp_err_t err = esp_console_run(s, &rc)) {
        case ESP_ERR_NOT_FOUND:
            printf("Unrecognized command: %s\n", s);
            break;
        case ESP_ERR_INVALID_ARG:
            break; // Command was empty
        case ESP_OK:
            if (rc != ESP_OK)
                printf("Command returned non-zero error code: 0x%x (%s)\n", rc, esp_err_to_name(rc));
            break;
        default:
            printf("Internal error: %s\n", esp_err_to_name(err));
            break;
        }

        fflush(stdout);
        linenoiseFree(s);
    }
}

Console::Console(const int p) : TCPServer(p, (TCPServer::service_t)service) {
    esp_console_config_t console_config = ESP_CONSOLE_CONFIG_DEFAULT();
    esp_console_init(&console_config);

    esp_console_register_help_command();

    const esp_console_cmd_t tilt_cmd = {
        .command = "tilt",
        .help = "Adjust tilt PID gains",
        .hint = nullptr,
        .func = tilt,
        .argtable = &tilt_args,
    };
    esp_console_cmd_register(&tilt_cmd);

    const esp_console_cmd_t speed_cmd = {
        .command = "speed",
        .help = "Adjust speed PID gains",
        .hint = nullptr,
        .func = speed,
        .argtable = &speed_args,
    };
    esp_console_cmd_register(&speed_cmd);

    const esp_console_cmd_t turn_cmd = {
        .command = "turn",
        .help = "Adjust turn PID gains",
        .hint = nullptr,
        .func = turn,
        .argtable = &turn_args,
    };
    esp_console_cmd_register(&turn_cmd);

    const esp_console_cmd_t pid_cmd = {
        .command = "pid",
        .help = "Display adjustable PID gains",
        .hint = nullptr,
        .func = pid,
        .argtable = &pid_args,
    };
    esp_console_cmd_register(&pid_cmd);

    const esp_console_cmd_t motor_cmd = {
        .command = "motor",
        .help = "Enable drive motors",
        .hint = nullptr,
        .func = motor,
        .argtable = &motor_args,
    };
    esp_console_cmd_register(&motor_cmd);
}
