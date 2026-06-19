/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   底盘机构层实现：麦克纳姆轮逆解 + 四轮速度环 PID + CAN 下发
  *
  *   从 robot_control.c 下沉而来，麦轮公式与 PID 参数逐字不变。
  ******************************************************************************
  */
#include "chassis.h"

#include "CAN_receive.h"   /* CAN_cmd_chassis / get_chassis_motor_measure_point / motor_measure_t */
#include "pid.h"
#include "motor_config.h"  /* 集中电机配置：CAN / ID / PID 参数 */

#define CHASSIS_RPM_SCALE      1000.0f   // 底盘速度 -> rpm 比例
#define CHASSIS_MOTOR_COUNT    4U        // M3508 底盘电机数(CAN1)

static float target_speed[CHASSIS_MOTOR_COUNT];      // 四轮目标转速(rpm)
static motor_pid_t chassis_pid[CHASSIS_MOTOR_COUNT]; // 四轮速度环 PID

// 四路底盘电机速度环 PID 初始化
void chassis_init(void)
{
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        pid_init(&chassis_pid[i], CHASSIS_SPD_KP, CHASSIS_SPD_KI, CHASSIS_SPD_KD,
                 CHASSIS_SPD_IMAX, CHASSIS_SPD_OUT);
    }
}

// 麦克纳姆轮逆解：底盘速度(vx 前后, vy 左右, wz 旋转) -> 四轮目标转速(rpm)
// speed_scale 为速度倍率(= 遥控 w 通道 + 1)
void chassis_set_velocity(float vx, float vy, float wz, float speed_scale)
{
    target_speed[0] = -(vx + vy + wz) * CHASSIS_RPM_SCALE * speed_scale; // 右前
    target_speed[1] =  (vx - vy - wz) * CHASSIS_RPM_SCALE * speed_scale; // 左前
    target_speed[2] = -(vx + vy - wz) * CHASSIS_RPM_SCALE * speed_scale; // 右后
    target_speed[3] =  (vx - vy + wz) * CHASSIS_RPM_SCALE * speed_scale; // 左后
}

// 失能/停车：四轮目标清零
void chassis_stop(void)
{
    chassis_set_velocity(0.0f, 0.0f, 0.0f, 1.0f);
}

// 底盘速度环 PID + 电流下发(CAN1 M3508)
void chassis_update(void)
{
    float output[CHASSIS_MOTOR_COUNT];

    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        const motor_measure_t *motor = get_chassis_motor_measure_point(i);
        output[i] = pid_calc(&chassis_pid[i], target_speed[i], motor->speed_rpm);
    }

    CAN_cmd_chassis((int16_t)output[0], (int16_t)output[1],
                    (int16_t)output[2], (int16_t)output[3]);
}
