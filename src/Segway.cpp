#include "Segway.h"

#include <driver/timer.h>
#include <freertos/task.h>

// TODO Arduino artifacts, where is esp-idf/freertos equivalent
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// Tilt (vertical balancing) angle PID
// This code runs every 5mS

float Segway::tiltPID() {
    const int mpu_gyro_scaling = 131; // internal units -> degree (FS_SEL=0)
    const int mpu_accel_scaling = 16384; // internal units -> g (AFS_SEL= 0)
    const float radians2degrees = 180 / M_PI;

    int16_t accelX, accelY, accelZ; // raw 3-axis accelerometer (internal unit)
    int16_t gyroX, gyroY, gyroZ; // raw 3-axis gyroscope (internal unit)

    // Retrieve raw 6-axis data from inertial sensor, normalize gyro angle to degrees
    mpu->getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
    Gyro_x = gyroX / mpu_gyro_scaling; // Angular velocity about yz-plane
    Gyro_y = gyroY / mpu_gyro_scaling; // TODO remove Gyro_[yz] if unused
    Gyro_z = gyroZ / mpu_gyro_scaling;
    Angle_x = atan2(accelY, accelZ) * radians2degrees;

    // Calculate (noise filtered) tilt angle from mpu raw data
    Angle = kalmanfilter.Kalman_Filter(Angle_x, Gyro_x);

    // PID calculation
    return tiltPIDOutput = tilt.Kp * (tiltSetPoint - Angle) + tilt.Kd * (0 - Gyro_x);
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
        robot->tiltPID();
        // if (tick % every20mS) turnPID();
        // if (tick % every50mS) speedPID();
        robot->setPWM();
    }
}

// Instrumentation methods for Bluetooth console

void Segway::stop() {
    tiltSetPoint = 0;
    speedSetPoint = 0;
    turnSetPoint = 0;

    left_motor->stop();
    right_motor->stop();
}

void Segway::resetPidCoefficients() {
    tilt = tiltPIDDefaults;
    speed = speedPIDDefaults;
    turn = turnPIDDefaults;
}

// Class constructor

Segway::Segway(Motor *lm,  Motor *rm, ESP32Encoder *le, ESP32Encoder *re, MPU6050 *m) {
    left_motor = lm;
    right_motor = rm;
    left_encoder = le;
    right_encoder = re;
    mpu = m;

    stop();
    resetPidCoefficients();

    xTaskCreate((TaskFunction_t)segwayProcess, "segway", configMINIMAL_STACK_SIZE * 4, this, 5, &task);
}
