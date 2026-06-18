/**
  ******************************************************************************
  * @file    imu_filter.c
  * @brief   加速度计零偏卡尔曼滤波实现
  ******************************************************************************
  */
#include "imu_filter.h"

/* 噪声参数(三轴共用常量) */
static const float Q_accel = 0.001f;     // 过程噪声 (加速度)
static const float Q_bias  = 0.00001f;   // 过程噪声 (零偏)
static const float R_accel_meas = 0.01f; // 测量噪声 (加速度计)

/* 每轴独立的协方差矩阵(修复原单 P 三轴共用导致的互相污染) */
static float P[IMU_FILTER_AXES][2][2] = {
    {{1.0f, 0.0f}, {0.0f, 1.0f}},
    {{1.0f, 0.0f}, {0.0f, 1.0f}},
    {{1.0f, 0.0f}, {0.0f, 1.0f}},
};

void kalman_filter_accel(float measured_accel, float *true_accel, float *bias, int axis)
{
    if (axis < 0 || axis >= IMU_FILTER_AXES) return;
    float (*Pa)[2] = P[axis];   /* 本轴协方差矩阵 */

    // 1. 预测步骤
    float F[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}}; // 状态转移矩阵
    float x_hat[2] = {*true_accel, *bias};        // 状态向量

    // 预测状态
    float x_hat_predicted[2];
    x_hat_predicted[0] = F[0][0] * x_hat[0] + F[0][1] * x_hat[1];
    x_hat_predicted[1] = F[1][0] * x_hat[0] + F[1][1] * x_hat[1];

    // 预测协方差矩阵
    float P_predicted[2][2];
    P_predicted[0][0] = F[0][0]*Pa[0][0]*F[0][0] + F[0][1]*Pa[1][0]*F[0][0] + F[0][0]*Pa[0][1]*F[1][0] + F[0][1]*Pa[1][1]*F[1][0] + Q_accel;
    P_predicted[0][1] = F[0][0]*Pa[0][0]*F[0][1] + F[0][1]*Pa[1][0]*F[0][1] + F[0][0]*Pa[0][1]*F[1][1] + F[0][1]*Pa[1][1]*F[1][1];
    P_predicted[1][0] = F[1][0]*Pa[0][0]*F[0][0] + F[1][1]*Pa[1][0]*F[0][0] + F[1][0]*Pa[0][1]*F[1][0] + F[1][1]*Pa[1][1]*F[1][0];
    P_predicted[1][1] = F[1][0]*Pa[0][0]*F[0][1] + F[1][1]*Pa[1][0]*F[0][1] + F[1][0]*Pa[0][1]*F[1][1] + F[1][1]*Pa[1][1]*F[1][1] + Q_bias;

    // 2. 更新步骤
    float H[1][2] = {{1.0f, 1.0f}}; // 测量矩阵
    float z = measured_accel;       // 测量值

    // 残差
    float y = z - (H[0][0] * x_hat_predicted[0] + H[0][1] * x_hat_predicted[1]);

    // 残差协方差
    float S = H[0][0]*P_predicted[0][0]*H[0][0] + H[0][1]*P_predicted[1][0]*H[0][0] + H[0][0]*P_predicted[0][1]*H[0][1] + H[0][1]*P_predicted[1][1]*H[0][1] + R_accel_meas;

    // 卡尔曼增益
    float K[2];
    K[0] = (P_predicted[0][0] * H[0][0] + P_predicted[0][1] * H[0][1]) / S;
    K[1] = (P_predicted[1][0] * H[0][0] + P_predicted[1][1] * H[0][1]) / S;

    // 更新状态估计
    *true_accel = x_hat_predicted[0] + K[0] * y;
    *bias       = x_hat_predicted[1] + K[1] * y;

    // 更新本轴协方差矩阵
    Pa[0][0] = (1 - K[0]*H[0][0]) * P_predicted[0][0] - K[0]*H[0][1]*P_predicted[1][0];
    Pa[0][1] = (1 - K[0]*H[0][0]) * P_predicted[0][1] - K[0]*H[0][1]*P_predicted[1][1];
    Pa[1][0] = -K[1]*H[0][0]*P_predicted[0][0] + (1 - K[1]*H[0][1]) * P_predicted[1][0];
    Pa[1][1] = -K[1]*H[0][0]*P_predicted[0][1] + (1 - K[1]*H[0][1]) * P_predicted[1][1];
}
