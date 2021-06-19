#include "Segway.h"

#include <driver/timer.h>
#include <driver/uart.h>
#include <esp_console.h>
#include <esp_log.h>
#include <freertos/task.h>
#include <linenoise/linenoise.h>
#include <string.h>

// TODO Arduino artifacts, where is esp-idf/freertos equivalent
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// Segway class is a singleton
Segway *robot = nullptr;

// Tilt (vertical balancing) angle PID
// This code runs every 5mS

#define UNUSED(x) (void)(x) // Ref. https://stackoverflow.com/questions/3599160

void Segway::serviceMotionApps() {}


void Segway::serviceMPU() {
    const int mpu_gyro_scaling = 131; // internal units -> degree (FS_SEL=0)
    const int mpu_accel_scaling = 16384; // internal units -> g (AFS_SEL= 0)
    UNUSED(mpu_accel_scaling); // Suppress compiler warning
    const float radians2degrees = 180 / M_PI;

    int16_t accelX, accelY, accelZ; // raw 3-axis accelerometer (internal unit)
    int16_t gyroX, gyroY, gyroZ; // raw 3-axis gyroscope (internal unit)

    // Retrieve raw 6-axis data from inertial sensor, normalize gyro angle to degrees
    mpu->getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
    Gyro_x = gyroX / mpu_gyro_scaling; // Angular velocity about yz-plane
    Gyro_y = gyroY / mpu_gyro_scaling; // TODO remove Gyro_[yz] if unused
    Gyro_z = gyroZ / mpu_gyro_scaling;
    Angle_x = atan2(accelY, accelZ) * radians2degrees;
}

float Segway::tiltPID() {

    // Calculate (noise filtered) tilt angle from mpu raw data
    Angle = kalmanfilter.Kalman_Filter(Angle_x, Gyro_x);
    tiltAcceleration = kalmanfilter.angle_dot;

    // PID calculation
    return tiltPIDOutput = tilt.Kp * (tiltSetPoint - Angle) + tilt.Kd * (0 - tiltAcceleration);
}

// Angular (turn/spin) velocity PID function
// Yields rotational difference between two motors
// This code runs every 20mS

float Segway::turnPID() {
    int spinonce = 0;
    float turnspeed = 0;
    float rotationratio = 0;

    // Are we going straight ahead
    if (turnSetPoint != 0) {
        // We are _not_ going straight ahead/back
        if (spinonce == 0) { // First time through only
            // TODO Bug This is accumulator for 50mS interrupt - Will be all over the place - Bug?
            turnspeed = abs(left_encoder->delta() + right_encoder->delta());
            spinonce++;
        }

        // Limit angular velocity depending if we are spinning (in place) or turning (while moving)
        if (speedSetPoint != 0) { // Turning while moving
            turnLimit = 3;
        } else { // Spinning in place
            turnLimit = 10;
        }

        rotationratio = 5 / turnspeed;
        if (rotationratio < 0.5) rotationratio = 0.5;
        if (rotationratio > 5) rotationratio = 5;
    } else {
        // We are going straight ahead/back
        rotationratio = 0.5; // Both wheels operate equally
        spinonce = 0;
        turnspeed = 0;
    }

    // Adjust setpoint with rotationratio, signed based on direction
    turnError += turnSetPoint < 0 ? -rotationratio : +rotationratio;
    if (turnSetPoint == 0) turnError = 0;

    // turnmin < turnError < turnLimit, limits depend if we are turning or spinning
    turnError = constrain(turnError, -turnLimit, turnLimit);

    // PID calculation
    // TODO PID calcuations in standard form

    return turnPIDOutput = -turnError * turn.Kp - Gyro_z * turn.Kd;
}

// Linear (forward/back) velocity PID function
// This code runs every 50mS

float Segway::speedPID() {
    
    // Encoder pulses since last invocation, crude overall distance estimate
    int d = left_encoder->delta() + right_encoder->delta();

    // TODO research digital filter terminology to properly describe this
    // (oem) Carry out low-pass filtering to slow down the speed difference and disturb the upright
    // As this code runs every 50mS, we can easily derive speed from distance travelled
    float s = velocity * 0.7  + d * 0.3;
    velocity = s;

    // Desired speed = accelerate by f=250 b=-250 on every call, with a speed limit
    // TODO speed PID math below does not make sense
    distance += s;
    distance += speedSetPoint;
    distance = constrain(distance, -3550, 3550); // TODO where do these numbers come from

    // PID calculation
    return speedPIDOutput = speed.Ki * (0.0 - distance) + speed.Kp * (0.0 - s);
}

// Computed motor speeds, direction reversal via controller bitbang
// TODO incerase PWM resolution 1to 1024 (all in hardware, should cost nothing)
// Note motors are mounted opposite each other, so must work in opposite directions to move together
// TODO motor controller also supports brake and stop - are these useful here?
// This code runs every 5mS

void Segway::setPWM() {

    // Raw motor PWM is superposition of our 3 stimuli; probably have embedded coefficients
    // (oem) speed and turn are interference for tilt term
    // TODO what are the measurement units PID function outputs - ultimately i am summing to a PWM value - dimensionless?
    leftMotorPWM  = (int)(-tiltPIDOutput - speedPIDOutput - turnPIDOutput); 
    rightMotorPWM = (int)(-tiltPIDOutput - speedPIDOutput + turnPIDOutput);
    leftMotorPWM  = constrain(leftMotorPWM, -255, 255);
    rightMotorPWM = constrain(rightMotorPWM, -255, 255);

    // If the robot is about to fall over, or already lying on its side, stop both motors
    // TODO original code has stanza for robot being picked up, refers to kalmanfilter.angle6
    if (abs(Angle) > 30) leftMotorPWM = rightMotorPWM = 0;

    // Set speed and direction on both motors
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
        robot->serviceMPU();
        robot->tiltPID();
        // if (!(robot->tick % every20mS)) robot->turnPID();
        // if (!(robot->tick % every50mS)) robot->speedPID();
        robot->setPWM();
    }
}

// Instrumentation methods for Bluetooth console

void Segway::stop() {
    speedSetPoint = 0;
    left_motor->stop();
    right_motor->stop();
}

Segway::Segway(Motor *lm,  Motor *rm, ESP32Encoder *le, ESP32Encoder *re, MPU6050 *m) {
    left_motor = lm;
    right_motor = rm;
    left_encoder = le;
    right_encoder = re;
    mpu = m;

    stop();

    xTaskCreate((TaskFunction_t)segwayProcess, "segway", configMINIMAL_STACK_SIZE * 4, this, 6, &task);
}

// Helm interface 

void Helm::service(const Helm *p, const int fd) {
    char c;

    static const char *TAG = "cockpit";

    // Design intent is to use numeric keypad as makeshift joystick
    while (recv(fd, &c, sizeof(c), 0) > 0) {
        ESP_LOGW(TAG, "%c", c);
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

void Telemetry::service(const Telemetry *p, const int fd) {
    while (true) {
        int x = printf("%.2f %.2f %.2f\n", robot->Angle, robot->tiltAcceleration, robot->tiltPIDOutput);

        if (x < 0) return;
        vTaskDelay(100 / portTICK_PERIOD_MS);
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

int Console::pid(int argc, char **argv) {
    printf("Tilt: Kp %.2f Ki %.2f Kd %.2f\n", robot->tilt.Kp, robot->tilt.Ki, robot->tilt.Kd);
    printf("Turn: Kp %.2f Ki %.2f Kd %.2f\n", robot->turn.Kp, robot->turn.Ki, robot->turn.Kd);
    printf("Speed: Kp %.2f Ki %.2f Kd %.2f\n", robot->speed.Kp, robot->speed.Ki, robot->speed.Kd);
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

// TODO linenoise hangs on startup while figuring out line length (over network)

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

    const esp_console_cmd_t pid_cmd = {
        .command = "pid",
        .help = "Display adjustable PID gains",
        .hint = nullptr,
        .func = pid,
        .argtable = arg_end(5),
    };
    esp_console_cmd_register(&pid_cmd);
}
