/**
  ******************************************************************************
  * @file    motor_config.h
  * @brief   全部电机的集中配置：CAN 总线 / CAN ID / 各环 pid_init 参数
  *
  *   调参只改本文件一处。每组 = 一个电机(或控制轴)的总线、ID 与角度/速度环参数。
  *   发球机构的目标位、方向、限流、死区、分阶段 PID 也集中在这里。
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
 * 两个 M3508 反相驱动同一轴：0x201 出 +I、0x202 出 -I；仅读 0x201 编码器。  */
#define ARM_BUS             2          /* CAN2 */
#define ARM_ID_A            0x201      /* +I 电机 */
#define ARM_ID_B            0x202      /* -I 电机 */
#define ARM_FB_ID           0x201      /* 反馈编码器所在 ID */

/* 击球臂 - 状态1：0 点附近保持。参数偏软，优先抑制零点抖动。 */
#define ARM_STATE1_POS             0.0f
#define ARM_STATE1_ANG_KP          1.2f
#define ARM_STATE1_ANG_KI          0.0f
#define ARM_STATE1_ANG_KD          0.0f
#define ARM_STATE1_ANG_IMAX        500.0f
#define ARM_STATE1_ANG_OUT         300.0f
#define ARM_STATE1_SPD_KP          5.0f
#define ARM_STATE1_SPD_KI          0.0f
#define ARM_STATE1_SPD_KD          0.08f
#define ARM_STATE1_SPD_IMAX        1000.0f
#define ARM_STATE1_SPD_OUT         2200.0f
#define ARM_STATE1_ANGLE_DZ        80.0f
#define ARM_STATE1_SPEED_DZ        15.0f
#define ARM_STATE1_SPEED_LIMIT     80.0f

/* 击球臂 - 状态2：蓄力到位并保持。 */
#define ARM_STATE2_POS             2000.583f
#define ARM_STATE2_DIR             (+1)
#define ARM_STATE2_ANG_KP          6.0f
#define ARM_STATE2_ANG_KI          0.0f
#define ARM_STATE2_ANG_KD          0.0f
#define ARM_STATE2_ANG_IMAX        1000.0f
#define ARM_STATE2_ANG_OUT         1000.0f
#define ARM_STATE2_SPD_KP          15.0f
#define ARM_STATE2_SPD_KI          0.0f
#define ARM_STATE2_SPD_KD          0.05f
#define ARM_STATE2_SPD_IMAX        4000.0f
#define ARM_STATE2_SPD_OUT         10000.0f
#define ARM_STATE2_ANGLE_DZ        0.0f
#define ARM_STATE2_SPEED_DZ        0.0f
#define ARM_STATE2_SPEED_LIMIT     300.0f

/* 击球臂 - 状态3：等待光电触发前的位置保持。 */
#define ARM_STATE3_HOLD_ANG_KP       2.5f
#define ARM_STATE3_HOLD_ANG_KI       0.0f
#define ARM_STATE3_HOLD_ANG_KD       0.0f
#define ARM_STATE3_HOLD_ANG_IMAX     800.0f
#define ARM_STATE3_HOLD_ANG_OUT      600.0f
#define ARM_STATE3_HOLD_SPD_KP       8.0f
#define ARM_STATE3_HOLD_SPD_KI       0.0f
#define ARM_STATE3_HOLD_SPD_KD       0.10f
#define ARM_STATE3_HOLD_SPD_IMAX     1800.0f
#define ARM_STATE3_HOLD_SPD_OUT      4500.0f
#define ARM_STATE3_HOLD_ANGLE_DZ     10.0f
#define ARM_STATE3_HOLD_SPEED_DZ     5.0f
#define ARM_STATE3_HOLD_SPEED_LIMIT  160.0f

/* 击球臂 - 状态3击打：开环电流 + 断流检测位置。 */
#define ARM_STATE3_STRIKE_CURRENT    16000.0f
#define ARM_STATE3_STRIKE_DIR        (-1)
#define ARM_STATE3_STRIKE_CUTOFF_POS (-1509.136f)

/* ====================== 电磁铁高度 / 抛球蓄力 (CAN2) ==================== *
 * 单个 M3508，ID 0x203，双环位置控制(移动到蓄力位并保持)。                */
#define TOSS_BUS            2          /* CAN2 */
#define TOSS_ID             0x203

/* 电磁铁高度 - 上电机械归零：开环正电流推向机械 0 点。 */
#define TOSS_STARTUP_HOME_MS       500U
#define TOSS_STARTUP_HOME_CURRENT  1500.0f

/* 电磁铁高度 - 状态1：0 点附近保持。 */
#define TOSS_STATE1_POS              0.0f
#define TOSS_STATE1_ANG_KP           1.0f
#define TOSS_STATE1_ANG_KI           0.0f
#define TOSS_STATE1_ANG_KD           0.0f
#define TOSS_STATE1_ANG_IMAX         500.0f
#define TOSS_STATE1_ANG_OUT          300.0f
#define TOSS_STATE1_SPD_KP           4.0f
#define TOSS_STATE1_SPD_KI           0.0f
#define TOSS_STATE1_SPD_KD           0.10f
#define TOSS_STATE1_SPD_IMAX         800.0f
#define TOSS_STATE1_SPD_OUT          1800.0f
#define TOSS_STATE1_ANGLE_DZ         60.0f
#define TOSS_STATE1_SPEED_DZ         10.0f
#define TOSS_STATE1_SPEED_LIMIT      80.0f

/* 电磁铁高度 - 状态2：拉齿条到蓄力位并保持。 */
#define TOSS_STATE2_POS              (-5500.595f)
#define TOSS_STATE2_DIR              (-1)
#define TOSS_STATE2_ANG_KP           2.0f
#define TOSS_STATE2_ANG_KI           0.0f
#define TOSS_STATE2_ANG_KD           2.0f
#define TOSS_STATE2_ANG_IMAX         1000.0f
#define TOSS_STATE2_ANG_OUT          1000.0f
#define TOSS_STATE2_SPD_KP           6.0f
#define TOSS_STATE2_SPD_KI           0.02f
#define TOSS_STATE2_SPD_KD           0.3f
#define TOSS_STATE2_SPD_IMAX         2500.0f
#define TOSS_STATE2_SPD_OUT          6000.0f
#define TOSS_STATE2_ANGLE_DZ         0.0f
#define TOSS_STATE2_SPEED_DZ         0.0f
#define TOSS_STATE2_SPEED_LIMIT      160.0f

/* 电磁铁高度 - 状态3：释放后保持进入发球瞬间的位置。 */
#define TOSS_STATE3_HOLD_ANG_KP       1.8f
#define TOSS_STATE3_HOLD_ANG_KI       0.0f
#define TOSS_STATE3_HOLD_ANG_KD       0.0f
#define TOSS_STATE3_HOLD_ANG_IMAX     800.0f
#define TOSS_STATE3_HOLD_ANG_OUT      700.0f
#define TOSS_STATE3_HOLD_SPD_KP       6.0f
#define TOSS_STATE3_HOLD_SPD_KI       0.01f
#define TOSS_STATE3_HOLD_SPD_KD       0.15f
#define TOSS_STATE3_HOLD_SPD_IMAX     1600.0f
#define TOSS_STATE3_HOLD_SPD_OUT      3500.0f
#define TOSS_STATE3_HOLD_ANGLE_DZ     5.0f
#define TOSS_STATE3_HOLD_SPEED_DZ     5.0f
#define TOSS_STATE3_HOLD_SPEED_LIMIT  120.0f

/* =========================== 发球机构公共参数 =========================== */
#define RM3508_CURRENT_MAX       16384.0f

/* 已经在目标附近时不强制再绕一圈，避免到位保持时被定向逻辑赶走 */
#define DIRECTED_TARGET_DZ       50.0f

/* CAN2 反馈数组(get_can2_motor_measure_point)下标，由 ID 推导，集中于此 */
#define ARM_FB_IDX          (ARM_FB_ID - 0x201)   /* = 0 */
#define TOSS_FB_IDX         (TOSS_ID   - 0x201)   /* = 2 */

#endif /* MOTOR_CONFIG_H */
