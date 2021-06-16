// Driver class for Osoyoo 2WD Balance Car Robot
// Ref. https://osoyoo.com/2018/08/01/introduction-of-osoyoo-2wd-balancing-car-robot/
// Ref. https://github.com/osoyoo/Osoyoo-development-kits

// Other vendors package the same kit but with different firmware, and different documentation
// Ref. https://www.elegoo.com/collections/robot-kits/products/elegoo-tumbller-self-balancing-robot-car
// Ref. https://wiki.keyestudio.com/Ks0193_keyestudio_Self-balancing_Car#Project_13:_Bluetooth_Control

#pragma once

#include "ESP32Encoder.h"
#include "KalmanFilter.h"
#include "Motor.h"
#include "Segway.h"
#include "TCPServer.h"

#include <MPU6050.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// TODO carefully think through fail-safe modes, especially for software errors and runaway conditions
// TODO single LED that shows balancing software in or out of sync
// TODO emergency stop button, a power switch would be better, need better placement

class Segway {
  public:

    Segway(Motor *left_motor,  Motor *right_motor,
            ESP32Encoder *left_encoder, ESP32Encoder *right_encoder,
            MPU6050 *mpu);

    // Timebase for all our digital filters, event to be raised every 5mS
    // TODO these definitions should be protected
    static const int handlerIntervalmS = 5; // milliSeconds
    TaskHandle_t task = nullptr;
    int tick = 0;
    float tiltPID();
    float turnPID();
    float speedPID();
    void setPWM();

    // Robot movement is controlled by directly setting these signed registers
    int tiltSetPoint = 0; // (angular position about X-axis) This is the "balance" part; leave at zero
    int turnSetPoint = 0; // (angular position about Z-axis) Heading
    int speedSetPoint = 0; // (linear velocity along Y-axis) stop=0, negative=reverse
    void stop(); // Resets the above to zero == dead stop

    // *** Instrumentation interface for Bluetooth console below this line ***

    // PID function coefficients
    // Ref. https://en.wikipedia.org/wiki/PID_controller

    typedef struct {
        float Kp, Ki, Kd;
    } pidCoefficients;

    // These default settings are recommended as a starting point by the vendor
    const pidCoefficients tiltPIDDefaults = {75.0, 0.0, 0.6};
    const pidCoefficients speedPIDDefaults = {5.2, 0.0, 0.0};
    const pidCoefficients turnPIDDefaults = {23.0, 0.0, 0.3};
    void resetPidCoefficients(); // Reset working registers to above defaults

    // Tilt (vertical balancing) angle PID, output updated every 5mS
    // Inertial measurement apparatus yields current angular position in 3 dimensions
    pidCoefficients tilt = tiltPIDDefaults;
    KalmanFilter kalmanfilter;
    float Gyro_x, Gyro_y, Gyro_z; // 3-axis gyroscope (degrees)
    float Angle_x; // Estimated (noisy) tilt angle (degrees) about yz-plane
    float Angle; // Tilt angle (degrees) (noise filtered)
    float tiltPIDOutput = 0;

    // Turn (angular/turn/spin) velocity PID function, output updated every 20mS
    pidCoefficients turn = turnPIDDefaults;
    int turnLimit = 0; // Upper boundary for turn angle
    float turnError = 0; // Error term for PID calculation
    float turnPIDOutput = 0;
    
    // Speed linear (forward/back) velocity PID, output updated every 50mS
    pidCoefficients speed = speedPIDDefaults;
    float velocity; // State variable for velocity PID calculation
    float distance; // Travelled since last PID calculation, estimated from encoders
    float speedPIDOutput = 0;

    // Computed motor controls, output updated every 5mS
    int leftMotorPWM = 0;
    int rightMotorPWM = 0;

  private:
    Motor *left_motor = nullptr;
    Motor *right_motor = nullptr;
    ESP32Encoder *left_encoder = nullptr;
    ESP32Encoder *right_encoder = nullptr;
    MPU6050 *mpu = nullptr;
};

// Helm interface

class Helm : public TCPServer {
  public:
    Helm(const int port, Segway *robot);

  private:
    Segway *robot = nullptr;
    static void service(const Helm *p, const int fd);
};

// Telemetry interface

class Telemetry : public TCPServer {
  public:
    Telemetry(const int port, const Segway *robot);

  private:
    const Segway *robot = nullptr;
    static void service(const Telemetry *p, const int fd);
};

// Console interface

class Console : public TCPServer {
  public:
    Console(const int port);

  private:
    int port = 0;
    static void service(const Console *p, const int fd);
};
