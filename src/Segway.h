// Driver class for Osoyoo 2WD Balance Car Robot
// Ref. https://osoyoo.com/2018/08/01/introduction-of-osoyoo-2wd-balancing-car-robot/
// Ref. https://github.com/osoyoo/Osoyoo-development-kits

// Based on public-domain driver supplied by product vendor
// Factored to partition functional logic from Bluetooth console support
// Many cleanups, namespace consistency, attempt to better annotate the code

// Other vendors package the same kit but with different firmware, and different documentation
// Ref. htt5ps://www.elegoo.com/collections/robot-kits/products/elegoo-tumbller-self-balancing-robot-car
// Ref. https://wiki.keyestudio.com/Ks0193_keyestudio_Self-balancing_Car#Project_13:_Bluetooth_Control

#pragma once

#include "ESP32Encoder.h"
#include "KalmanFilter.h"
#include "Motor.h"

#include <MPU6050.h>
#include <freertos/task.h>

// TODO review methods/members for const arguments
// TODO some are float, some are double -- why?
// TODO make sure floating point operations are kept to a minimum, really expensive on UNO
// TODO carefully think through fail-safe modes, especially for software errors and runaway conditions
// TODO single LED that shows balancing software in or out of sync
// TODO emergency stop button, a power switch would be better

class Segway {
  public:

    Segway(Motor *left_motor,  Motor *right_motor,
            ESP32Encoder *left_encoder, ESP32Encoder *right_encoder,
            MPU6050 *mpu);

    // Timebase for all our digital filters, event to be raised every 5mS
    // TODO these definitions should be private
    static const int handlerIntervalmS = 5; // milliSeconds
    int tick = 0;

    // Robot movement is controlled by directly setting these signed registers
    int tiltSetPoint = 0; // (angular position about X-axis) This is the "balance" part; leave at zero
    int turnSetPoint = 0; // (angular position about Z-axis) Heading
    int speedSetPoint = 0; // (linear velocity along Y-axis) stop=0, negative=reverse
    void stop(); // Resets the above to zero == dead stop

    // *** Instrumentation interface for Bluetooth console below this line ***
    // Note most of this data is visible im main.cpp, as it created all these objects

    typedef struct {
        float Kp, Ki, Kd;
    } pidCoefficients;

    // PID function coefficients, adjustable through console
    // These default settings are recommended as a starting point by the vendor
    const pidCoefficients tiltPIDDefaults = {40.0, 0.0, 0.6};
    const pidCoefficients speedPIDDefaults = {5.2, 0.0, 0.0};
    const pidCoefficients turnPIDDefaults = {23.0, 0.0, 0.3};
    void resetPidCoefficients(); // Reset working registers to above defualts

    // Latest MPU sample
    const int mpu_gyro_scaling = 131; // internal units / degree (FS_SEL=0)
    const int mpu_accel_scaling = 16384; // internal units / g (AFS_SEL= 0)
    int16_t accelX, accelY, accelZ; // 3-axis accelerometer (internal unit)
    int16_t gyroX, gyroY, gyroZ; // 3-axis gyroscope (internal unit)
    float Gyro_x, Gyro_y, Gyro_z; // 3-axis gyroscope (degrees)
    float Angle_x; // Estimated (noisy) tilt angle (degrees) about yz-plane
    float Angle; // Tilt angle (degrees) (noise filtered)

    // Tilt (vertical balancing) angle PID, output updated every 5mS
    // Inertial measurement apparatus yields current angular position in 3 dimensions
    pidCoefficients tiltPIDGains = tiltPIDDefaults;
    KalmanFilter kalmanfilter;
    float tiltPIDOutput = 0;

    // Turn (angular/turn/spin) velocity PID function, output updated every 20mS
    pidCoefficients turnPIDGains = turnPIDDefaults;
    int turnLimit = 0; // Upper boundary for turn angle
    float turnError = 0; // Error term for PID calculation
    float turnPIDOutput = 0;
    
    // Speed linear (forward/back) velocity PID, output updated every 50mS
    pidCoefficients speedPIDGains = speedPIDDefaults;
    float speed; // State variable for velocity PID calculation
    float distance; // Travelled since last PID calculation, estimated from encoders
    float speedPIDOutput = 0;

    // Computed motor controls, output updated every 5mS
    float leftMotorPWM = 0;
    float rightMotorPWM = 0;

    TaskHandle_t task = nullptr;
    float tiltPID();
    float turnPID();
    float speedPID();
    void setPWM();

  private:
    Motor *left_motor = nullptr;
    Motor *right_motor = nullptr;
    ESP32Encoder *left_encoder = nullptr;
    ESP32Encoder *right_encoder = nullptr;
    MPU6050 *mpu = nullptr;
};
