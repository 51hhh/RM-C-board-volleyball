/**
  ******************************************************************************
  * @file    striker.h
  * @brief   发球击球机构层 (CAN2 上 3 个 M3508)
  *
  *   - 击球臂：两个电机反相驱动同一根轴(0x201 +I / 0x202 -I)，仅读 0x201 编码器。
  *   - 电磁铁高度：单电机(0x203)，双环位置控制。
  *   - 状态一：两轴 0 位等待，电磁铁吸合。
  *   - 状态二：两轴按规定方向到蓄力位等待，电磁铁持续吸合。
  *   - 状态三：电磁铁释放抛球，光电开关第一次触发 300ms 后臂开环击打，
  *             经过击球检测位后断流。
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
    STRIKER_IDLE    = 0,   // 上档：两轴回 0 等待，电磁铁吸合
    STRIKER_PREPARE = 1,   // 中档：两轴按规定方向到蓄力位，电磁铁吸合
    STRIKER_SERVE   = 2,   // 下档：电磁铁释放，第一次光电触发 300ms 后击打并过位断流
};

typedef struct {
    int     mode;
    float   arm_pos;             // 击球臂多圈位置(度)
    float   arm_target;
    float   arm_single_angle;    // 当前单圈角度(度)
    int32_t arm_turns;           // 跨圈累计
    float   arm_strike_cutoff;
    int16_t arm_current;
    float   toss_pos;            // 电磁铁高度电机多圈位置(度)
    float   toss_target;
    float   toss_single_angle;
    int32_t toss_turns;
    int16_t toss_current;
    uint8_t magnet_on;
    uint8_t serve_current_on;
    uint8_t magnet_pin_output;    // 1=输出模式，0=高阻输入
    uint8_t magnet_pin_cmd;       // 1=命令高电平，0=命令低电平
    uint8_t magnet_pin_level;     // HAL 读回 PI7 实际电平
    uint8_t photo_pin_level;      // HAL 读回 PI6 实际电平
    uint8_t photo_active;         // 1=当前光电有效
    uint8_t photo_stable_active;  // 1=光电有效电平已稳定
    uint8_t photo_armed;          // 1=已稳定释放，允许识别下一次触发
    uint8_t photo_trigger_count;  // 本次 SERVE 中是否已识别第一次触发(0/1)
    uint8_t photo_strike_pending; // 1=第一次触发后等待 300ms 击打
    uint8_t photo_strike_fired;   // 1=本次 SERVE 已经触发过击打
} striker_debug_t;

/* 击球臂/电磁铁高度位置环 PID 初始化 */
void striker_init(void);

/* 每个 1kHz 控制拍调用：读多圈角度 -> 发球状态机 -> 电流计算 -> CAN2 反相下发 */
void striker_update(uint32_t now_ms);

/* 设置发球状态(STRIKER_IDLE/PREPARE/SERVE)，由 robot_control 的遥控解析调用 */
void striker_set_mode(int mode);

/* 电磁铁控制接口。on=true 吸合(锁住蓄力)，false 释放。 */
void striker_magnet_set(bool on);

/* 调试快照：用于串口遥测记录多圈位置并回填 striker.c 的标定常量。 */
void striker_get_debug(striker_debug_t *debug);

#ifdef __cplusplus
}
#endif

#endif /* STRIKER_H */
