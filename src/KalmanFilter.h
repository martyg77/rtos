/*KalmanFilter.h KalmanFilter.cpp - Library for Balance robot code*/
// Copy taken from Osoyoo source, chinese comments included

#pragma once

#include <math.h>
#include <stdint.h>

// TODO clean up KalmanFilter library, source from generic math library
// TODO document overview of inertial sensing process, provide references - from noisy redundant data to stable attitude measurement
// Ref. https://mjwhite8119.github.io/Robots/mpu6050

class KalmanFilter {
  public:
    
    // Kalman filter coefficients
    const float dt = 0.005; // Filter sampling time (Seconds)
    const float Q_angle = 0.001; // Gyroscope noise covariance
    const float Q_gyro = 0.005; // Gyroscope drift noise covariance
    const float R_angle = 0.5; // Accelerometer covariance
    const float C_0 = 1.0;

    float filter(float angle_m, float gyro_m);
    void Yiorderfilter(float angle_m, float gyro_m, float dt, float K1);

    // TODO obsolete
    // void Angletest(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, 
    //               float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1);
    // float Gyro_x, Gyro_y, Gyro_z; // Latest raw MPU samples (degrees)

    // float accelz = 0;

    float angle_accel;    
  private:
    float angle = 0.0;
    float angle6 = 0;
    
    float angle_err;
    float q_bias = 0;
    float Pdot[4] = {0, 0, 0, 0};
    float P[2][2] = {{1, 0}, {0, 1}};
    float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
};
