/*
KalmanFilter.h KalmanFilter.cpp - Library for Balance robot code.
*/

#include "KalmanFilter.h"

/********************************yijielvbo********************************/
void KalmanFilter::Yiorderfilter(float angle_m, float gyro_m, float dt, float K1) {
    angle6 = K1 * angle_m + (1 - K1) * (angle6 + gyro_m * dt);
    // return angle6;
}

/********************************kalman********************************/

float KalmanFilter::Kalman_Filter(float angle_m, float gyro_m) {
    angle += (gyro_m - q_bias) * dt;
    angle_err = angle_m - angle;
    Pdot[0] = Q_angle - P[0][1] - P[1][0];
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = Q_gyro;
    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    angle += K_0 * angle_err; //���ŽǶ�
    q_bias += K_1 * angle_err;
    angle_dot = gyro_m - q_bias; //���Ž��ٶ�
    return angle;
}

/********************************Angle test********************************/
void Angletest(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, 
                            float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1) {

    //��ת�Ƕ�Z�����
    if (gz > 32768) gz -= 65536; // TODO compiler warning comparison always
    
    float angleAx = atan2(ax, az) * 180 / M_PI; //������x��н�
 //   Yiorderfilter(angleAx, Gyro_y, dt, K1); //һ���˲�

    
    // TODO not sure what this does
    // This method updates kalmanfilter.angle6
    // const float K1 = 0.05; // Weight of accelerometer values (Kalman gain)
    // float Angle_y = atan2(accelX, accelZ) * 180 / PI;
    // kalmanfilter.Yiorderfilter(Angle_y, Gyro_y, dt, K1);

}
