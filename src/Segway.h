// Driver class for Osoyoo 2WD Balance Car Robot
// Ref. https://osoyoo.com/2018/08/01/introduction-of-osoyoo-2wd-balancing-car-robot/
// Ref. https://github.com/osoyoo/Osoyoo-development-kits

// Other vendors package the same kit but with different firmware, and different documentation
// Ref. https://www.elegoo.com/collections/robot-kits/products/elegoo-tumbller-self-balancing-robot-car
// Ref. https://wiki.keyestudio.com/Ks0193_keyestudio_Self-balancing_Car

// Reference and background information
// Ref. https://en.wikipedia.org/wiki/Inverted_pendulum
// Ref. https://robotics.ee.uwa.edu.au/theses/2003-Balance-Ooi.pdf

#pragma once

#include "ESP32Encoder.h"
#include "KalmanFilter.h"
#include "Motor.h"
#include "TCPServer.h"

#include <MPU6050.h>
#include <argtable3/argtable3.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// TODO carefully think through fail-safe modes, especially for software errors and runaway conditions
// TODO single LED that shows balancing software in or out of sync
// TODO emergency stop button, a power switch would be better, need better placement

class Segway {
  public:
    Segway(Motor *left_motor, Motor *right_motor,
           ESP32Encoder *left_encoder, ESP32Encoder *right_encoder,
           MPU6050 *mpu);

    // Timebase for all our digital filters, event to be raised every 5mS
    // TODO these definitions should be protected
    static const int handlerIntervalmS = 5; // milliSeconds
    TaskHandle_t task = nullptr;
    int tick = 0;
    float tiltPID();
    float speedPID();
    float turnPID();
    void setPWM();

    // PID setpoints, ref. https://en.wikipedia.org/wiki/PID_controller
    // Robot movement is controlled by directly setting these signed registers
    int tiltSetPoint = 0; // (angular position about X-axis) This is the "balance" part; leave at zero
    int speedSetPoint = 0; // (linear velocity along Y-axis) stop=0, negative=reverse
    int turnSetPoint = 0; // (angular position about Z-axis) Heading
    void stop(); // Resets the above to zero == dead stop

    // PID function coefficients, must be non-negative
    typedef struct {
        float Kp, Ki, Kd;
    } pidCoefficients;

    const pidCoefficients tiltPIDDefaults = {75.0, 0.0, 0.6};
    const pidCoefficients speedPIDDefaults = {5.2, 0.25, 0.0};
    const pidCoefficients turnPIDDefaults = {23.0, 0.0, 0.3};

    // Tilt (vertical balancing) angle PID, output updated every 5mS
    // Inertial measurement apparatus yields current angular position in 3 dimensions
    pidCoefficients tilt = tiltPIDDefaults;
    KalmanFilter kalman;
    float GyroX, GyroY, GyroZ; // Unfiltered 3-axis angular velocity (degrees/Sec)
    float tiltAngle = 0; // Noise-filtered tilt angle (degrees)
    float tiltAcceleration = 0; // Noise-filtered tilt velocity (degrees/Sec)
    float tiltControl = 0;

    // Speed linear (forward/back) velocity PID, output updated every 50mS
    pidCoefficients speed = speedPIDDefaults;
    float speedOMeter = 0; // Linear component for velocity PID calculation
    float speedErrorIntegral = 0;
    float speedControl = 0;

    // Turn (angular/turn/spin) velocity PID function, output updated every 20mS
    pidCoefficients turn = turnPIDDefaults;
    float turnControl = 0;

    // Computed motor controls, output updated every 5mS
    bool motorEnable = false;
    int leftMotorPWM = 0;
    int rightMotorPWM = 0;

  private:
    Motor *left_motor = nullptr;
    Motor *right_motor = nullptr;
    ESP32Encoder *left_encoder = nullptr;
    ESP32Encoder *right_encoder = nullptr;
    MPU6050 *mpu = nullptr;
};

// Segway class is a singleton
extern Segway *robot;

// Helm
// Numeric keypad interface for basic motion control
// TODO integrate proportional controller (joystick, RC radio) for motion control

class Helm : public TCPServer {
  public:
    Helm(const int port) : TCPServer(port, (TCPServer::service_t)service) {}

  private:
    static void service();
};

// Telemetry
// Used to monitor performance data in real-time as the robot operates

// feedgnuplot frontend works well to visulalize data on Unix
// Ref. https://github.com/dkogan/feedgnuplot
// e.g. nc segway 4444 | feedgnuplot -- stream

class Telemetry : public TCPServer {
  public:
    Telemetry(const int port) : TCPServer(port, (TCPServer::service_t)service) {}

  private:
    static void service();
};

// Console
// Basic command-line interface
// TODO linenoise editor/history support (Hangs on startup while figuring out line length)

class Console : public TCPServer {
  public:
    Console(const int port);

  private:
    static void service();
    static void parser();
    static int pid(int argc, char **argv);
    static int motor(int argc, char **argv);
    static int tilt(int argc, char **argv);
    static int speed(int argc, char **argv);
    static int turn(int argc, char **argv);
};
