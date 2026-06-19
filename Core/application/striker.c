/**
  ******************************************************************************
  * @file    striker.c
  * @brief   发球击球机构层实现 (CAN2 三个 M3508)
  *
  *   双环级联 = 复用 pid.c 的 motor_pid_t/pid_calc：角度外环输出作速度内环设定，
  *   速度内环反馈取"本拍多圈位置增量"。在 1kHz 固定拍执行(速度量纲恒为 度/ms)，
  *   修掉原击球臂工程自由循环下速度量纲漂移的问题。死区复刻原 PID_Cal_Limt。
  *
  *   PID 参数与目标位沿用原工程作起点，1kHz 下需上板重新整定。
  ******************************************************************************
  */
#include "striker.h"

#include "main.h"          /* HAL_GPIO(电磁铁) */
#include "CAN_receive.h"   /* CAN_cmd_can2 / get_can2_motor_measure_point */
#include "pid.h"           /* motor_pid_t / pid_init / pid_calc */
#include "motor_config.h"  /* 集中电机配置：CAN / ID / PID 参数 */

#include <math.h>

/* ===== 电磁铁释放接口(预留) =====
 * 配好 CubeMX 引脚后，取消下面两行注释并填入实际端口/引脚即可启用：
 *   #define MAGNET_GPIO_Port  GPIOx
 *   #define MAGNET_Pin        GPIO_PIN_x
 * 未定义时 striker_magnet_set 为空操作(不会误写引脚)。 */
/* #define MAGNET_GPIO_Port  GPIOB */
/* #define MAGNET_Pin        GPIO_PIN_13 */

/* CAN2 反馈索引 ARM_FB_IDX / TOSS_FB_IDX 已集中到 motor_config.h */

/* ===== 目标位(度，多圈累加坐标；起点取自原工程，需整定) ===== */
#define ARM_IDLE_POS        0.0f
#define ARM_PREPARE_POS    -1450.0f   // 臂后摆蓄势
#define ARM_STRIKE_POS      1200.0f   // 挥击到位
#define TOSS_IDLE_POS       0.0f
#define TOSS_CHARGE_POS     200.0f    // 抛球蓄力位(占位，需整定)
#define STRIKE_DELAY_MS     165U      // 电磁铁释放后到挥击的延时(原值)

/* ===== 级联死区(复刻原 PID_Cal_Limt：|err|≤dz 则该环清零，防到位抖动) ===== */
#define ANGLE_DZ            50.0f
#define SPEED_DZ            10.0f

/* 单轴位置跟踪状态(PID 不在此，按工况由调用方选 gentle/strong) */
typedef struct {
    float   pos_abs;     // 多圈累加位置(度)
    float   pos_last;    // 上一拍 pos_abs(算速度)
    float   angle_last;  // 上一拍单圈角度(度，算多圈增量)
    float   target;      // 目标位
    uint8_t inited;      // 首拍初始化
} axis_t;

static axis_t s_arm;
static axis_t s_toss;

/* 臂：常规定位(gentle) + 强力挥击(strong) 两套级联 PID；抛球一套 */
static motor_pid_t arm_angle_pid,  arm_speed_pid;
static motor_pid_t arm_strike_angle_pid, arm_strike_speed_pid;
static motor_pid_t toss_angle_pid, toss_speed_pid;

static int      s_mode = STRIKER_IDLE;  // 发球状态
static uint8_t  s_arm_strong = 0;       // 臂是否用强力 PID(挥击阶段)
static uint32_t s_serve_stamp = 0;      // 进入发球的时刻(电磁铁释放计时)

/* 多圈角度累加：单圈 0~360 回绕取 |Δ|≤180 的真实增量(等价原 Motor_Angle_Cal) */
static void axis_track_angle(axis_t *ax, float real_angle)
{
    if (!ax->inited) {
        ax->angle_last = real_angle;
        ax->pos_last   = ax->pos_abs;
        ax->inited     = 1;
        return;
    }
    float d = real_angle - ax->angle_last;
    if (d > 180.0f)       d -= 360.0f;
    else if (d < -180.0f) d += 360.0f;
    ax->pos_abs   += d;
    ax->angle_last = real_angle;
}

/* 双环级联一步：角度外环 -> 速度内环，各带死区。返回电流设定。
 * apid/spid 由调用方选择(臂可切常规/强力)。 */
static float cascade_step(axis_t *ax, float target,
                          motor_pid_t *apid, motor_pid_t *spid,
                          float a_dz, float s_dz)
{
    // 角度外环：误差 = 目标 - 多圈位置
    float a_err = target - ax->pos_abs;
    float out_a = pid_calc(apid, target, ax->pos_abs);
    if (fabsf(a_err) <= a_dz) { out_a = 0.0f; apid->i_out = 0.0f; }   // 死区

    // 速度内环：反馈 = 本拍位置增量(度/拍)，设定 = 角度环输出
    float speed = ax->pos_abs - ax->pos_last;
    ax->pos_last = ax->pos_abs;
    float s_err = out_a - speed;
    float out_s = pid_calc(spid, out_a, speed);
    if (fabsf(s_err) <= s_dz) { out_s = 0.0f; spid->i_out = 0.0f; }   // 死区

    return out_s;
}

/* 发球状态机：设定两轴目标位 + 电磁铁 + 臂强力切换。
 * 结构改自原 type0/1/2：机构由"电机释放"改为"蓄力 + 电磁铁释放"。 */
static void serve_state_machine(uint32_t now)
{
    switch (s_mode) {
    case STRIKER_PREPARE:
        s_arm.target  = ARM_PREPARE_POS;   // 臂后摆蓄势
        s_toss.target = TOSS_CHARGE_POS;   // 抛球电机移到蓄力位并保持
        striker_magnet_set(true);          // 电磁铁吸合，锁住蓄力
        s_arm_strong  = 0;
        s_serve_stamp = 0;
        break;

    case STRIKER_SERVE:
        if (s_serve_stamp == 0) {          // 进入发球瞬间
            s_serve_stamp = now;
            striker_magnet_set(false);     // 电磁铁释放 -> 放能/抛球
        }
        s_toss.target = TOSS_CHARGE_POS;   // 抛球保持(放能由电磁铁完成)
        if (now - s_serve_stamp >= STRIKE_DELAY_MS) {
            s_arm.target = ARM_STRIKE_POS;  // 延时后强力挥击
            s_arm_strong = 1;
        } else {
            s_arm.target = ARM_PREPARE_POS; // 延时窗口内保持蓄势
            s_arm_strong = 0;
        }
        break;

    case STRIKER_IDLE:
    default:
        s_arm.target  = ARM_IDLE_POS;
        s_toss.target = TOSS_IDLE_POS;
        striker_magnet_set(false);
        s_arm_strong  = 0;
        s_serve_stamp = 0;
        break;
    }
}

void striker_init(void)
{
    // 击球臂：常规定位(温和)
    pid_init(&arm_angle_pid, ARM_ANG_KP, ARM_ANG_KI, ARM_ANG_KD, ARM_ANG_IMAX, ARM_ANG_OUT);
    pid_init(&arm_speed_pid, ARM_SPD_KP, ARM_SPD_KI, ARM_SPD_KD, ARM_SPD_IMAX, ARM_SPD_OUT);
    // 击球臂：强力挥击
    pid_init(&arm_strike_angle_pid, ARM_STK_ANG_KP, ARM_STK_ANG_KI, ARM_STK_ANG_KD, ARM_STK_ANG_IMAX, ARM_STK_ANG_OUT);
    pid_init(&arm_strike_speed_pid, ARM_STK_SPD_KP, ARM_STK_SPD_KI, ARM_STK_SPD_KD, ARM_STK_SPD_IMAX, ARM_STK_SPD_OUT);
    // 抛球蓄力
    pid_init(&toss_angle_pid, TOSS_ANG_KP, TOSS_ANG_KI, TOSS_ANG_KD, TOSS_ANG_IMAX, TOSS_ANG_OUT);
    pid_init(&toss_speed_pid, TOSS_SPD_KP, TOSS_SPD_KI, TOSS_SPD_KD, TOSS_SPD_IMAX, TOSS_SPD_OUT);

    s_mode = STRIKER_IDLE;
}

void striker_update(uint32_t now_ms)
{
    // 1) 读两轴多圈角度(臂读 0x201，抛球读 0x203)
    axis_track_angle(&s_arm,  get_can2_motor_measure_point(ARM_FB_IDX)->real_angle);
    axis_track_angle(&s_toss, get_can2_motor_measure_point(TOSS_FB_IDX)->real_angle);

    // 2) 发球状态机：设定目标 + 电磁铁 + 强力切换
    serve_state_machine(now_ms);

    // 3) 双环级联得电流(臂在挥击阶段切强力 PID)
    motor_pid_t *arm_a = s_arm_strong ? &arm_strike_angle_pid : &arm_angle_pid;
    motor_pid_t *arm_s = s_arm_strong ? &arm_strike_speed_pid : &arm_speed_pid;
    float i_arm  = cascade_step(&s_arm,  s_arm.target,  arm_a, arm_s, ANGLE_DZ, SPEED_DZ);
    float i_toss = cascade_step(&s_toss, s_toss.target, &toss_angle_pid, &toss_speed_pid, ANGLE_DZ, SPEED_DZ);

    // 4) CAN2 下发：击球臂双电机反相(0x201 +I / 0x202 -I)，抛球(0x203)
    CAN_cmd_can2((int16_t)i_arm, (int16_t)(-i_arm), (int16_t)i_toss);
}

void striker_set_mode(int mode)
{
    if (mode == STRIKER_IDLE || mode == STRIKER_PREPARE || mode == STRIKER_SERVE) {
        s_mode = mode;
    }
}

void striker_magnet_set(bool on)
{
#ifdef MAGNET_GPIO_Port
    HAL_GPIO_WritePin(MAGNET_GPIO_Port, MAGNET_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;   // 引脚未配置：预留接口，待 CubeMX 配好后定义 MAGNET_GPIO_Port/Pin 启用
#endif
}
