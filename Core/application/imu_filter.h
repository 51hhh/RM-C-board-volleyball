/**
  ******************************************************************************
  * @file    imu_filter.h
  * @brief   加速度计零偏卡尔曼滤波 (单轴二维状态: [真实加速度, 零偏])
  *
  * 每个轴(X/Y/Z)维护独立的协方差矩阵，互不干扰。
  ******************************************************************************
  */
#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_FILTER_AXES 3   /* X=0, Y=1, Z=2 */

/* 卡尔曼滤波更新(单轴)
 *   measured_accel : 本轴加速度计原始测量值
 *   true_accel     : [in/out] 滤波后真实加速度估计
 *   bias           : [in/out] 零偏估计
 *   axis           : 轴索引 0..2，用于选择独立的协方差矩阵
 */
void kalman_filter_accel(float measured_accel, float *true_accel, float *bias, int axis);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FILTER_H */
