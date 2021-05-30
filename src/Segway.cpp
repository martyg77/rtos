#include "Segway.h"

// Rotary shaft encoder interrupt handlers
// TODO this comment block belongs elswhere

// Hall-effect encoder magnet has 13 poles, gear reduction to wheel axle is 30:1
// Therefore both encoder pinA and pinB each provide 13*30=390 pulses/axle revolution

// Supplied wheels are ~65mm diameter, ~205mm linear displacement per axle revolution 

// One signal (pinA or pinB) sufficient (theoretically) for determining distance only
// Differential mode (using both pins) provides distance and direction (forward/reverse)

// Encoder signals are notorious for noise: this may significantly impact calculations
// Differential mode provides redundancy, which can be leveraged for noise reduction

// TODO Arduino artifacts, where is esp-idf/freertos equivalent
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// Tilt (vertical balancing) angle PID
// This code runs every 5mS
// Ref. https://mjwhite8119.github.io/Robots/mpu6050

// TODO document overview of inertial sensing process, provide references - from noisy redundant data to stable attitude measurement

// Kalman filter coefficients
// TODO what do these magic kalman filter constants do, where did these numbers come from, what are othes using
const float dt = Segway::handlerIntervalmS * 1e-3;
const float Q_angle = 0.001, Q_gyro = 0.005; // Confidence of angle data and confidence of angular velocity data
const float R_angle = 0.5, C_0 = 1;
const float K1 = 0.05; // Weight of accelerometer values

double Segway::tiltPID() {
    
    // Retrieve raw 6-axis data from inertial sensor
    mpu->getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ); 
    
    // Kalman filtering, angle computation
    // TODO find a standard Kalman filter library
    // TODO Kalman filter code requires careful review - Is this butied in new MPC library, what about MotionApps
    kalmanfilter.Angletest(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
    
    // PID calculation
    // TODO PID calcuations in standard form
    return tiltPIDOutput = tiltPIDGains.Kp * kalmanfilter.angle + tiltPIDGains.Kd * kalmanfilter.Gyro_x;
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


    return turnPIDOutput = -turnError * turnPIDGains.Kp - kalmanfilter.Gyro_z * turnPIDGains.Kd;
}

// Linear (forward/back) velocity PID function
// This code runs every 50mS

double Segway::speedPID() {
    
    // Encoder pulses since last invocation, crude overall distance estimate
    int d = left_encoder->delta() + right_encoder->delta();

    // TODO research digital filter terminology to properly describe this
    // (oem) Carry out low-pass filtering to slow down the speed difference and disturb the upright
    // As this code runs every 50mS, we can easily derive speed from distance travelled
    float s = speed * 0.7  + d * 0.3;
    speed = s;

    // Desired speed = accelerate by f=250 b=-250 on every call, with a speed limit
    // TODO speed PID math below does not make sense
    distance += s;
    distance += speedSetPoint;
    distance = constrain(distance, -3550, 3550); // TODO where do these numbers come from

    // PID calculation
    return speedPIDOutput = speedPIDGains.Ki * (0.0 - distance) + speedPIDGains.Kp * (0.0 - s);
}

// Computed motor speeds, direction reversal via controller bitbang
// Note motors are mounted opposite each other, so must work in opposite directions to move together
// TODO motor controller also supports brake and stop - are these useful here?
// This code runs every 5mS

void Segway::setPWM() {

    // Raw motor PWM is superposition of our 3 stimuli; probably have embedded coefficients
    // (oem) speed and turn are interference for tilt term
    // TODO what are the measurement units PID function outputs - ultimately i am summing to a PWM value - dimensionless?
    leftMotorPWM = -tiltPIDOutput - speedPIDOutput - turnPIDOutput; // TODO convert float -> int
    rightMotorPWM = -tiltPIDOutput - speedPIDOutput + turnPIDOutput;
    leftMotorPWM = constrain(leftMotorPWM, -255, 255);
    rightMotorPWM = constrain(rightMotorPWM, -255, 255);

    // If the robot is about to fall over, or already lying on its side, stop both motors
    // TODO original code has stanza for robot being picked up, refers to kalmanfilter.angle6
    if (abs(kalmanfilter.angle) > 30) leftMotorPWM = rightMotorPWM = 0;

    // Set speed and direction on both motors
    left_motor->run(leftMotorPWM);
    right_motor->run(rightMotorPWM);
}

// Main interrupt handler body
// This procedure to be called every 5mS

const int every20mS = 20 / Segway::handlerIntervalmS; // Used to schedule turn PID
const int every50mS = 50 / Segway::handlerIntervalmS; // Used to schedule speed PID
static int tick = 0;

void Segway::handler5mS() {
    tick = (tick + 1) % (every20mS * every50mS); // Prevent integer overflow
    tiltPID();
//  if (tick % every20mS) turnPID();
//  if (tick % every50mS) speedPID();
    setPWM();
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
    tiltPIDGains = tiltPIDDefaults;
    speedPIDGains = speedPIDDefaults;
    turnPIDGains = turnPIDDefaults;
}

// Class constructor

Segway::Segway(Motor *lm,  Motor *rm, Encoder *le, Encoder *re, MPU6050 *m) {
    left_motor = lm;
    right_motor = rm;
    left_encoder = le;
    right_encoder = re;
    mpu = m;

    stop();
    resetPidCoefficients();
}
