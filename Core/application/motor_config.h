/**
  ******************************************************************************
  * @file    motor_config.h
  * @brief   全部电机的集中配置：CAN 总线 / CAN ID / 各环 pid_init 参数
  *
  *   调参只改本文件一处。每组 = 一个电机(或控制轴)的总线、ID 与角度/速度环参数。
  *   范围仅含 id/can/PID；死区、发球目标位、麦轮 scale、遥控增益、wz_pid 等仍在各模块。
  *
  *   注：参数沿用原工程作起点，1kHz 控制拍下仍需上板整定。
  ******************************************************************************
  */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* ============================ 底盘 4× M3508 ============================ *
 * CAN1，ID 0x201~0x204，单速度环(麦轮)。四轮共用同一组参数。               */
#define CHASSIS_BUS         1          /* CAN1 (注：CAN_receive.h 的 CHASSIS_CAN=hcan1 是 HAL 句柄别名) */
#define CHASSIS_ID_BASE     0x201      /* 连续 4 个：0x201..0x204 */
#define CHASSIS_SPD_KP      3.5f
#define CHASSIS_SPD_KI      0.01f
#define CHASSIS_SPD_KD      0.0f
#define CHASSIS_SPD_IMAX    30000.0f
#define CHASSIS_SPD_OUT     16384.0f

/* ============================ 击球臂 (CAN2) ============================ *
 * 两个 M3508 反相驱动同一轴：0x201 出 +I、0x202 出 -I；仅读 0x201 编码器。  *
 * 双环级联(角度外环→速度内环)，另备一组"强力挥击"参数。                    */
#define ARM_BUS             2          /* CAN2 */
#define ARM_ID_A            0x201      /* +I 电机 */
#define ARM_ID_B            0x202      /* -I 电机 */
#define ARM_FB_ID           0x201      /* 反馈编码器所在 ID */
/* 常规定位 - 角度环 */
#define ARM_ANG_KP          6.0f
#define ARM_ANG_KI          0.0f
#define ARM_ANG_KD          0.0f
#define ARM_ANG_IMAX        1000.0f
#define ARM_ANG_OUT         1000.0f
/* 常规定位 - 速度环 */
#define ARM_SPD_KP          15.0f
#define ARM_SPD_KI          0.0f
#define ARM_SPD_KD          0.05f
#define ARM_SPD_IMAX        4000.0f
#define ARM_SPD_OUT         10000.0f
/* 强力挥击 - 角度环 */
#define ARM_STK_ANG_KP      2.0f
#define ARM_STK_ANG_KI      0.0f
#define ARM_STK_ANG_KD      1.0f
#define ARM_STK_ANG_IMAX    4500.0f
#define ARM_STK_ANG_OUT     6000.0f
/* 强力挥击 - 速度环 */
#define ARM_STK_SPD_KP      3.5f
#define ARM_STK_SPD_KI      0.0f
#define ARM_STK_SPD_KD      1.5f
#define ARM_STK_SPD_IMAX    4500.0f
#define ARM_STK_SPD_OUT     13300.0f

/* ========================== 抛球蓄力 (CAN2) =========================== *
 * 单个 M3508，ID 0x203，双环位置控制(移动到蓄力位并保持)。                */
#define TOSS_BUS            2          /* CAN2 */
#define TOSS_ID             0x203
/* 角度环 */
#define TOSS_ANG_KP         2.0f
#define TOSS_ANG_KI         0.0f
#define TOSS_ANG_KD         2.0f
#define TOSS_ANG_IMAX       1000.0f
#define TOSS_ANG_OUT        1000.0f
/* 速度环 */
#define TOSS_SPD_KP         6.0f
#define TOSS_SPD_KI         0.02f
#define TOSS_SPD_KD         0.3f
#define TOSS_SPD_IMAX       2500.0f
#define TOSS_SPD_OUT        6000.0f

/* CAN2 反馈数组(get_can2_motor_measure_point)下标，由 ID 推导，集中于此 */
#define ARM_FB_IDX          (ARM_FB_ID - 0x201)   /* = 0 */
#define TOSS_FB_IDX         (TOSS_ID   - 0x201)   /* = 2 */

#endif /* MOTOR_CONFIG_H */
