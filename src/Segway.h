// Driver class for Osoyoo 2WD Balance Car Robot
// Ref. https://osoyoo.com/2018/08/01/introduction-of-osoyoo-2wd-balancing-car-robot/
// Ref. https://github.com/osoyoo/Osoyoo-development-kits

// Based on public-domain driver supplied by product vendor
// Factored to partition functional logic from Bluetooth console support
// Many cleanups, namespace consistency, attempt to better annotate the code

// Other vendors package the same kit but with different firmware, and different documentation
// Ref. https://www.elegoo.com/collections/robot-kits/products/elegoo-tumbller-self-balancing-robot-car
// Ref. https://wiki.keyestudio.com/Ks0193_keyestudio_Self-balancing_Car#Project_13:_Bluetooth_Control

#pragma once

#include "Encoder.h"
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
            Encoder *left_encoder, Encoder *right_encoder,
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
        double Kp;
        double Ki;
        double Kd;
    } pidCoefficients;

    // PID function coefficients, adjustable through console
    // These default settings are recommended as a starting point by the vendor
    const pidCoefficients tiltPIDDefaults = {90.0, 0.0, 2.5};
    const pidCoefficients speedPIDDefaults = {5.2, 0.0, 0.0};
    const pidCoefficients turnPIDDefaults = {23.0, 0.0, 0.3};
    void resetPidCoefficients(); // Reset working registers to above defualts

    // Tilt (vertical balancing) angle PID, output updated every 5mS
    // Inertial measurement apparatus yields current angular position in 3 dimensions
    pidCoefficients tiltPIDGains = tiltPIDDefaults;
    int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ; // Latest raw input from MPU6050
    KalmanFilter kalmanfilter;
    // TODO clean up KalmanFilter library, source from generic math library
    double tiltPIDOutput = 0;

    // Turn (angular/turn/spin) velocity PID function, output updated every 20mS
    pidCoefficients turnPIDGains = turnPIDDefaults;
    int turnLimit = 0; // Upper boundary for turn angle
    float turnError = 0; // Error term for PID calculation
    float turnPIDOutput = 0;
    
    // Speed linear (forward/back) velocity PID, output updated every 50mS
    pidCoefficients speedPIDGains = speedPIDDefaults;
    float speed; // State variable for velocity PID calculation
    float distance; // Travelled since last PID calculation, estimated from encoders
    double speedPIDOutput = 0;

    // Computed motor controls, output updated every 5mS
    double leftMotorPWM = 0;
    double rightMotorPWM = 0;

    TaskHandle_t task = nullptr;
    double tiltPID();
    float turnPID();
    double speedPID();
    void setPWM();

  private:
    Motor *left_motor = nullptr;
    Motor *right_motor = nullptr;
    Encoder *left_encoder = nullptr;
    Encoder *right_encoder = nullptr;
    MPU6050 *mpu = nullptr; 
};
