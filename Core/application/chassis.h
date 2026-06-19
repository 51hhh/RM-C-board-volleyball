/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   底盘机构层：麦克纳姆轮逆解 + 四轮速度环 PID + CAN 电流下发
  *
  *   只负责"按给定底盘速度(vx 前后, vy 左右, wz 旋转)驱动四个 M3508"，
  *   不涉及任何控制策略/状态机(那些在 robot_control.c)。
  ******************************************************************************
  */
#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

/* 四路速度环 PID 初始化 */
void chassis_init(void);

/* 麦克纳姆轮逆解：底盘速度 -> 四轮目标转速。
 *   vx          : 前后
 *   vy          : 左右平移
 *   wz          : 旋转
 *   speed_scale : 速度倍率 */
void chassis_set_velocity(float vx, float vy, float wz, float speed_scale);

/* 四轮目标清零(失能/停车) */
void chassis_stop(void);

/* 速度环 PID 计算 + CAN 电流下发(每控制拍调用) */
void chassis_update(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
