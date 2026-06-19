/**
  ******************************************************************************
  * @file    striker.h
  * @brief   发球击球机构层 (CAN2 上 3 个 M3508)
  *
  *   - 击球臂：两个电机反相驱动同一根轴(0x201 +I / 0x202 -I)，仅读 0x201 编码器；
  *     双环级联(角度外环 -> 速度内环)位置控制。
  *   - 抛球蓄力：单电机(0x203)，同样双环位置控制(移动到蓄力位并保持)。
  *   - 发球时序：准备(臂后摆蓄势 + 抛球蓄力 + 电磁铁吸合) -> 发球(电磁铁释放放能
  *     -> 延时 -> 臂切强力 PID 挥击)。电磁铁释放为预留接口。
  *
  *   电机控制复用 pid.c 的 motor_pid_t/pid_calc：每轴两个 PID 串成级联。
  ******************************************************************************
  */
#ifndef STRIKER_H
#define STRIKER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 发球状态(由遥控设置) */
enum {
    STRIKER_IDLE    = 0,   // 空闲：保持当前位
    STRIKER_PREPARE = 1,   // 准备：臂后摆蓄势 + 抛球蓄力 + 电磁铁吸合锁住
    STRIKER_SERVE   = 2,   // 发球：电磁铁释放 -> 延时 -> 臂强力挥击
};

/* 3 轴(臂角度环/速度环 + 抛球角度环/速度环 + 臂强力组) PID 初始化 */
void striker_init(void);

/* 每个 1kHz 控制拍调用：读多圈角度 -> 发球状态机 -> 双环级联 -> CAN2 反相下发 */
void striker_update(uint32_t now_ms);

/* 设置发球状态(STRIKER_IDLE/PREPARE/SERVE)，由 robot_control 的遥控解析调用 */
void striker_set_mode(int mode);

/* 电磁铁释放接口(预留)。on=true 吸合(锁住蓄力)，false 释放(放能/抛球)。
 * 实际动作需在 CubeMX 配好引脚并在 striker.c 定义 MAGNET_GPIO_Port/MAGNET_Pin。 */
void striker_magnet_set(bool on);

#ifdef __cplusplus
}
#endif

#endif /* STRIKER_H */
