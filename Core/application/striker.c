/**
  ******************************************************************************
  * @file    striker.c
  * @brief   发球击球机构层实现 (CAN2 三个 M3508)
  *
  *   遥控三档：
  *     上档 IDLE    : 臂/电磁铁高度电机回 0 等待，电磁铁吸合。
  *     中档 PREPARE : 臂按规定方向慢速转到蓄力位，高度电机到蓄力位，电磁铁持续吸合。
  *     下档 SERVE   : 电磁铁释放抛球，光电开关第二次触发并延时后臂开环击打；
  *                    经过击球检测位后断流，靠重力回落。
  *
  *   两个位置轴都使用多圈累加坐标(度)，避免 3508 单圈编码器跨圈后只看到很小角差。
  *   状态二/三的运动目标在进入状态时锁存为规定方向上的等效多圈目标，不走最短路径。
  ******************************************************************************
  */
#include "striker.h"

#include "main.h"          /* HAL_GPIO(电磁铁) */
#include "CAN_receive.h"   /* CAN_cmd_can2 / get_can2_motor_measure_point */
#include "pid.h"           /* motor_pid_t / pid_init / pid_calc */
#include "motor_config.h"  /* 集中电机配置：CAN / ID / PID 参数 */

#include <math.h>

/* ===== 电磁铁控制：PI7 =====
 * 吸合时主动驱动，释放时切高阻，避免驱动板输入极性/上拉下拉不确定时被 MCU 强驱。 */
#define MAGNET_GPIO_Port    GPIOI
#define MAGNET_Pin          GPIO_PIN_7
#define MAGNET_ACTIVE_LEVEL GPIO_PIN_SET
#define MAGNET_RELEASE_HIZ  0U

/* ===== 光电开关：PI6 =====
 * 默认按 NPN/开漏输出处理：内部上拉，低电平表示被触发。
 * 若传感器输出高电平有效，把 PHOTO_ACTIVE_LEVEL 改为 GPIO_PIN_SET。 */
#define PHOTO_GPIO_Port         GPIOI
#define PHOTO_Pin               GPIO_PIN_6
#define PHOTO_ACTIVE_LEVEL      GPIO_PIN_RESET
#define PHOTO_TRIGGER_DEBOUNCE_MS 20U
#define PHOTO_STRIKE_DELAY_MS   0U

/* CAN2 反馈索引 ARM_FB_IDX / TOSS_FB_IDX 已集中到 motor_config.h */

/* ===== 标定量(度，多圈累加坐标；上板记录后回填) ===== */
#define ARM_IDLE_POS              0.0f      // 状态一实测等待位：arm_pos=0.000
#define ARM_PREPARE_POS        2000.583f   // 状态二实测蓄力位：arm_pos=5344.583
#define ARM_STRIKE_CUTOFF_POS    (-1509.136f) // 状态三实测击球断流位：arm_pos=-1109.136
#define TOSS_IDLE_POS             0.0f      // 状态一作为电磁铁高度电机相对零点
#define TOSS_CHARGE_POS        (-5500.595f) // 状态二相对状态一位移：-296.543 - 3883.052

/* ===== 规定运动方向：+1=多圈角度增大，-1=多圈角度减小 ===== */
#define ARM_PREPARE_DIR          (+1)
#define ARM_STRIKE_DIR           (-1)
#define TOSS_CHARGE_DIR          (-1)

/* ===== 电流/限幅 ===== */
#define ARM_STRIKE_CURRENT       16000.0f   // 状态三开环击打电流，最大不要超过 16384
#define RM3508_CURRENT_MAX       16384.0f
#define ARM_PREPARE_SPEED_LIMIT  300.0f     // 状态二击球臂蓄力速度上限；原外环上限为 1000
#define ARM_PREPARE_ANGLE_DZ     0.0f       // 蓄力保持不能用大死区，否则到位附近无保持力矩
#define ARM_PREPARE_SPEED_DZ     0.0f
#define TOSS_CHARGE_SPEED_LIMIT  160.0f     // 状态二电磁铁高度轴低速拉皮筋，防止瞬时速度过大脱磁
#define TOSS_CHARGE_ANGLE_DZ     0.0f       // 蓄力保持不能用大死区，否则负载会把位置拉回去
#define TOSS_CHARGE_SPEED_DZ     0.0f

/* 上电后先给电磁铁高度电机一个小电流，把机构推回机械 0 点，再把软件坐标清零。
 * 当前只处理高度电机；击球臂不自动推零，避免意外动作。 */
#define TOSS_STARTUP_HOME_MS      1000U
#define TOSS_STARTUP_HOME_CURRENT 900.0f

/* 已经在目标附近时不强制再绕一圈，避免到位保持时被定向逻辑赶走 */
#define DIRECTED_TARGET_DZ       50.0f

/* ===== 级联死区(复刻原 PID_Cal_Limt：|err|≤dz 则该环清零，防到位抖动) ===== */
#define ANGLE_DZ            50.0f
#define SPEED_DZ            10.0f

/* 单轴位置跟踪状态(PID 状态单独保存，多圈坐标用于目标和过位判断) */
typedef struct {
    float   pos_abs;     // 多圈累加位置(度)
    float   pos_last;    // 上一拍 pos_abs(算速度)
    float   angle_last;  // 上一拍单圈角度(度，算多圈增量)
    float   target;      // 目标位
    int32_t turns;       // 单圈编码器跨圈次数(调试观察用)
    uint8_t inited;      // 首拍初始化
} axis_t;

static axis_t s_arm;
static axis_t s_toss;

/* 臂/电磁铁高度：位置等待用级联 PID；状态三击打臂改为开环电流。 */
static motor_pid_t arm_angle_pid,  arm_speed_pid;
static motor_pid_t toss_angle_pid, toss_speed_pid;

static int      s_mode = STRIKER_IDLE;      // 当前发球状态
static uint8_t  s_mode_entry = 1;           // 下一拍执行状态进入动作
static uint8_t  s_idle_hold_current = 0;    // 逆向切回状态一时保持当前位置，不主动回零
static uint8_t  s_serve_current_on = 0;     // 状态三是否仍给击打电流
static uint8_t  s_magnet_on = 0;
static uint8_t  s_magnet_pin_output = 0xFFU;
static uint8_t  s_magnet_pin_cmd = 0;
static uint8_t  s_magnet_pin_level = 0;
static uint8_t  s_photo_pin_level = 0;
static uint8_t  s_photo_active = 0;
static uint8_t  s_photo_active_last = 0;
static uint8_t  s_photo_trigger_count = 0;
static uint8_t  s_photo_strike_pending = 0;
static uint8_t  s_photo_strike_fired = 0;
static uint32_t s_photo_last_trigger_stamp = 0;
static uint32_t s_photo_second_trigger_stamp = 0;
static uint8_t  s_startup_home_started = 0;
static uint8_t  s_startup_home_done = 0;
static uint32_t s_startup_home_stamp = 0;
static float    s_arm_strike_cutoff = ARM_STRIKE_CUTOFF_POS;
static int16_t  s_arm_current_cmd = 0;
static int16_t  s_toss_current_cmd = 0;

/* 多圈角度累加：单圈 0~360 回绕取 |Δ|≤180 的真实增量(等价原 Motor_Angle_Cal)。 */
static void axis_track_angle(axis_t *ax, float real_angle)
{
    if (!ax->inited) {
        ax->angle_last = real_angle;
        ax->pos_last   = ax->pos_abs;
        ax->inited     = 1;
        return;
    }
    float d = real_angle - ax->angle_last;
    if (d > 180.0f) {
        d -= 360.0f;
        ax->turns--;
    } else if (d < -180.0f) {
        d += 360.0f;
        ax->turns++;
    }
    ax->pos_abs   += d;
    ax->angle_last = real_angle;
}

static float clampf(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static int16_t clamp_current(float current)
{
    current = clampf(current, -RM3508_CURRENT_MAX, RM3508_CURRENT_MAX);
    return (int16_t)current;
}

static void pid_clear(motor_pid_t *pid)
{
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
    pid->p_out  = 0.0f;
    pid->i_out  = 0.0f;
    pid->d_out  = 0.0f;
    pid->output = 0.0f;
}

static void clear_axis_pid(void)
{
    pid_clear(&arm_angle_pid);
    pid_clear(&arm_speed_pid);
    pid_clear(&toss_angle_pid);
    pid_clear(&toss_speed_pid);
}

static void photo_sensor_init(void)
{
#ifdef PHOTO_GPIO_Port
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = PHOTO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = (PHOTO_ACTIVE_LEVEL == GPIO_PIN_RESET) ? GPIO_PULLUP : GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PHOTO_GPIO_Port, &GPIO_InitStruct);
#endif
}

static uint8_t photo_sensor_read_active(void)
{
#ifdef PHOTO_GPIO_Port
    GPIO_PinState state = HAL_GPIO_ReadPin(PHOTO_GPIO_Port, PHOTO_Pin);
    s_photo_pin_level = (state == GPIO_PIN_SET) ? 1U : 0U;
    s_photo_active = (state == PHOTO_ACTIVE_LEVEL) ? 1U : 0U;
    return s_photo_active;
#else
    s_photo_pin_level = 0U;
    s_photo_active = 0U;
    return 0U;
#endif
}

static void photo_serve_reset(void)
{
    s_photo_active_last = photo_sensor_read_active();
    s_photo_trigger_count = 0U;
    s_photo_strike_pending = 0U;
    s_photo_strike_fired = 0U;
    s_photo_last_trigger_stamp = 0U;
    s_photo_second_trigger_stamp = 0U;
}

static void photo_serve_update(uint32_t now_ms)
{
    uint8_t active = photo_sensor_read_active();

    if (active && !s_photo_active_last &&
        (s_photo_last_trigger_stamp == 0U ||
         now_ms - s_photo_last_trigger_stamp >= PHOTO_TRIGGER_DEBOUNCE_MS)) {
        s_photo_last_trigger_stamp = now_ms;

        if (s_photo_trigger_count < 2U) {
            s_photo_trigger_count++;
        }

        if (s_photo_trigger_count == 2U &&
            !s_photo_strike_pending &&
            !s_photo_strike_fired) {
            s_photo_strike_pending = 1U;
            s_photo_second_trigger_stamp = now_ms;
        }
    }

    s_photo_active_last = active;

    if (s_photo_strike_pending &&
        now_ms - s_photo_second_trigger_stamp >= PHOTO_STRIKE_DELAY_MS) {
        s_photo_strike_pending = 0U;
        s_photo_strike_fired = 1U;
        s_serve_current_on = 1U;
        clear_axis_pid();
    }
}

/* 把 base_pos 换算成从 current_pos 出发、沿 dir 方向会遇到的等效多圈目标。
 * 这一步只在进入状态时锁存，避免目标随当前位置滚动导致永远追不到。 */
static float directed_target(float current_pos, float base_pos, int dir)
{
    float target = base_pos;

    if (fabsf(target - current_pos) <= DIRECTED_TARGET_DZ) {
        return target;
    }

    if (dir > 0) {
        while (target < current_pos) {
            target += 360.0f;
        }
    } else if (dir < 0) {
        while (target > current_pos) {
            target -= 360.0f;
        }
    }

    return target;
}

static uint8_t axis_passed(float pos, float threshold, int dir)
{
    if (dir >= 0) {
        return pos >= threshold;
    }
    return pos <= threshold;
}

static void axis_zero(axis_t *ax)
{
    ax->pos_abs = 0.0f;
    ax->pos_last = 0.0f;
    ax->turns = 0;
}

/* 双环级联一步：角度外环 -> 速度内环，各带死区。返回电流设定。 */
static float cascade_step(axis_t *ax, float target,
                          motor_pid_t *apid, motor_pid_t *spid,
                          float a_dz, float s_dz,
                          float speed_limit)
{
    // 角度外环：误差 = 目标 - 多圈位置
    float a_err = target - ax->pos_abs;
    float out_a = pid_calc(apid, target, ax->pos_abs);
    if (fabsf(a_err) <= a_dz) { out_a = 0.0f; apid->i_out = 0.0f; }   // 死区
    if (speed_limit > 0.0f) {
        out_a = clampf(out_a, -speed_limit, speed_limit);
    }

    // 速度内环：反馈 = 本拍位置增量(度/拍)，设定 = 角度环输出
    float speed = ax->pos_abs - ax->pos_last;
    ax->pos_last = ax->pos_abs;
    float s_err = out_a - speed;
    float out_s = pid_calc(spid, out_a, speed);
    if (fabsf(s_err) <= s_dz) { out_s = 0.0f; spid->i_out = 0.0f; }   // 死区

    return out_s;
}

static void enter_mode(void)
{
    switch (s_mode) {
    case STRIKER_PREPARE:
        s_idle_hold_current = 0;
        s_arm.target  = directed_target(s_arm.pos_abs,  ARM_PREPARE_POS, ARM_PREPARE_DIR);
        s_toss.target = directed_target(s_toss.pos_abs, TOSS_CHARGE_POS, TOSS_CHARGE_DIR);
        s_serve_current_on = 0;
        s_photo_strike_pending = 0U;
        s_photo_strike_fired = 0U;
        striker_magnet_set(true);
        break;

    case STRIKER_SERVE:
        s_idle_hold_current = 0;
        s_toss.target = s_toss.pos_abs;    // 高度电机不换位置，只保持进入发球瞬间的位置
        s_arm.target = s_arm.pos_abs;      // 等待光电第二次触发前继续抱住击球臂
        s_arm_strike_cutoff = directed_target(s_arm.pos_abs, ARM_STRIKE_CUTOFF_POS, ARM_STRIKE_DIR);
        s_serve_current_on = 0;
        photo_serve_reset();
        striker_magnet_set(false);
        clear_axis_pid();
        break;

    case STRIKER_IDLE:
    default:
        if (s_idle_hold_current) {
            s_arm.target = s_arm.pos_abs;
            s_toss.target = s_toss.pos_abs;
        } else {
            s_arm.target  = ARM_IDLE_POS;
            s_toss.target = TOSS_IDLE_POS;
        }
        s_serve_current_on = 0;
        s_photo_strike_pending = 0U;
        s_photo_strike_fired = 0U;
        striker_magnet_set(true);
        break;
    }

    s_mode_entry = 0;
}

/* 发球状态机：只负责目标/磁铁/开环阶段，不直接下发 CAN。 */
static void serve_state_machine(uint32_t now_ms)
{
    if (s_mode_entry) {
        enter_mode();
    }

    switch (s_mode) {
    case STRIKER_PREPARE:
        striker_magnet_set(true);
        break;

    case STRIKER_SERVE:
        striker_magnet_set(false);
        photo_serve_update(now_ms);
        if (s_serve_current_on &&
            axis_passed(s_arm.pos_abs, s_arm_strike_cutoff, ARM_STRIKE_DIR)) {
            s_serve_current_on = 0;
            s_photo_strike_pending = 0U;
        }
        break;

    case STRIKER_IDLE:
    default:
        striker_magnet_set(true);
        break;
    }
}

void striker_init(void)
{
    // 击球臂：常规定位(温和)
    pid_init(&arm_angle_pid, ARM_ANG_KP, ARM_ANG_KI, ARM_ANG_KD, ARM_ANG_IMAX, ARM_ANG_OUT);
    pid_init(&arm_speed_pid, ARM_SPD_KP, ARM_SPD_KI, ARM_SPD_KD, ARM_SPD_IMAX, ARM_SPD_OUT);
    // 电磁铁高度
    pid_init(&toss_angle_pid, TOSS_ANG_KP, TOSS_ANG_KI, TOSS_ANG_KD, TOSS_ANG_IMAX, TOSS_ANG_OUT);
    pid_init(&toss_speed_pid, TOSS_SPD_KP, TOSS_SPD_KI, TOSS_SPD_KD, TOSS_SPD_IMAX, TOSS_SPD_OUT);

    s_mode = STRIKER_IDLE;
    s_mode_entry = 1;
    s_idle_hold_current = 0;
    s_arm.target = ARM_IDLE_POS;
    s_toss.target = TOSS_IDLE_POS;
    s_startup_home_started = 0;
    s_startup_home_done = 0;
    s_startup_home_stamp = 0;
    photo_sensor_init();
    photo_serve_reset();
    striker_magnet_set(true);
}

void striker_update(uint32_t now_ms)
{
    // 1) 读两轴多圈角度(臂读 0x201，抛球读 0x203)
    axis_track_angle(&s_arm,  get_can2_motor_measure_point(ARM_FB_IDX)->real_angle);
    axis_track_angle(&s_toss, get_can2_motor_measure_point(TOSS_FB_IDX)->real_angle);

    // 2) 上电高度电机机械归零：小电流推 1 秒，然后把高度多圈坐标清零
    if (!s_startup_home_done) {
        if (!s_startup_home_started) {
            s_startup_home_started = 1;
            s_startup_home_stamp = now_ms;
        }

        striker_magnet_set(true);
        s_arm_current_cmd = 0;
        s_toss_current_cmd = clamp_current(TOSS_STARTUP_HOME_CURRENT);
        CAN_cmd_can2(0, 0, s_toss_current_cmd);

        if (now_ms - s_startup_home_stamp >= TOSS_STARTUP_HOME_MS) {
            axis_zero(&s_toss);
            s_toss.target = TOSS_IDLE_POS;
            s_startup_home_done = 1;
            s_mode_entry = 1;
            clear_axis_pid();
        }
        return;
    }

    // 3) 发球状态机：设定/锁存目标 + 电磁铁 + 开环阶段
    serve_state_machine(now_ms);

    // 4) 得电流：SERVE 等光电期间臂仍位置保持；击打过位断流后给 0 电流靠重力回落
    float i_arm = 0.0f;
    if (s_mode == STRIKER_SERVE && s_serve_current_on) {
        i_arm = ARM_STRIKE_CURRENT * (float)ARM_STRIKE_DIR;
        s_arm.pos_last = s_arm.pos_abs;     // 开环期间仍刷新速度基准，避免回位置环时速度突变
    } else if (s_mode == STRIKER_SERVE && s_photo_strike_fired) {
        i_arm = 0.0f;
        s_arm.pos_last = s_arm.pos_abs;     // 断流回落期间不再位置保持
    } else {
        uint8_t arm_hold_loaded = (s_mode == STRIKER_PREPARE ||
                                   (s_mode == STRIKER_SERVE && !s_serve_current_on));
        float arm_speed_limit = arm_hold_loaded ? ARM_PREPARE_SPEED_LIMIT : 0.0f;
        float arm_angle_dz = arm_hold_loaded ? ARM_PREPARE_ANGLE_DZ : ANGLE_DZ;
        float arm_speed_dz = arm_hold_loaded ? ARM_PREPARE_SPEED_DZ : SPEED_DZ;
        i_arm = cascade_step(&s_arm, s_arm.target, &arm_angle_pid, &arm_speed_pid,
                             arm_angle_dz, arm_speed_dz, arm_speed_limit);
    }

    float toss_speed_limit = (s_mode == STRIKER_PREPARE) ? TOSS_CHARGE_SPEED_LIMIT : 0.0f;
    float toss_angle_dz = (s_mode == STRIKER_PREPARE) ? TOSS_CHARGE_ANGLE_DZ : ANGLE_DZ;
    float toss_speed_dz = (s_mode == STRIKER_PREPARE) ? TOSS_CHARGE_SPEED_DZ : SPEED_DZ;
    float i_toss = cascade_step(&s_toss, s_toss.target, &toss_angle_pid, &toss_speed_pid,
                                toss_angle_dz, toss_speed_dz, toss_speed_limit);

    s_arm_current_cmd  = clamp_current(i_arm);
    s_toss_current_cmd = clamp_current(i_toss);

#ifdef STRIKER_CALIB_NO_DRIVE
    s_arm_current_cmd = 0;
    s_toss_current_cmd = 0;
#endif

    // 5) CAN2 下发：击球臂双电机反相(0x201 +I / 0x202 -I)，抛球(0x203)
    CAN_cmd_can2(s_arm_current_cmd, (int16_t)(-s_arm_current_cmd), s_toss_current_cmd);
}

void striker_set_mode(int mode)
{
    if (mode == STRIKER_IDLE || mode == STRIKER_PREPARE || mode == STRIKER_SERVE) {
        /* 遥控三档从下档回上档时物理上会经过中档。
         * 已经发球后的 SERVE -> PREPARE 视为过渡噪声，避免逆向切换时重新蓄力。 */
        if (s_mode == STRIKER_SERVE && mode == STRIKER_PREPARE) {
            return;
        }

        if (s_mode != mode) {
            if (mode == STRIKER_IDLE &&
                (s_mode == STRIKER_PREPARE || s_mode == STRIKER_SERVE)) {
                s_idle_hold_current = 1;
            }
            s_mode = mode;
            s_mode_entry = 1;
            clear_axis_pid();
        }
    }
}

static void magnet_gpio_output(void)
{
#ifdef MAGNET_GPIO_Port
    if (s_magnet_pin_output == 1U) {
        return;
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = MAGNET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MAGNET_GPIO_Port, &GPIO_InitStruct);
    s_magnet_pin_output = 1U;
#endif
}

static void magnet_gpio_hiz(void)
{
#ifdef MAGNET_GPIO_Port
    if (s_magnet_pin_output == 0U) {
        return;
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = MAGNET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MAGNET_GPIO_Port, &GPIO_InitStruct);
    s_magnet_pin_output = 0U;
#endif
}

void striker_magnet_set(bool on)
{
#ifdef MAGNET_GPIO_Port
    s_magnet_on = on ? 1U : 0U;
    GPIO_PinState state = on ? MAGNET_ACTIVE_LEVEL :
                          ((MAGNET_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    s_magnet_pin_cmd = (state == GPIO_PIN_SET) ? 1U : 0U;
    if (on) {
        magnet_gpio_output();
        HAL_GPIO_WritePin(MAGNET_GPIO_Port, MAGNET_Pin, state);
    } else {
        HAL_GPIO_WritePin(MAGNET_GPIO_Port, MAGNET_Pin, state);
#if MAGNET_RELEASE_HIZ
        magnet_gpio_hiz();
#else
        magnet_gpio_output();
#endif
    }
    s_magnet_pin_level = (HAL_GPIO_ReadPin(MAGNET_GPIO_Port, MAGNET_Pin) == GPIO_PIN_SET) ? 1U : 0U;
#else
    (void)on;   // 引脚未配置：预留接口，待 CubeMX 配好后定义 MAGNET_GPIO_Port/Pin 启用
#endif
}

void striker_get_debug(striker_debug_t *debug)
{
    if (debug == NULL) {
        return;
    }

    debug->mode = s_mode;
    debug->arm_pos = s_arm.pos_abs;
    debug->arm_target = s_arm.target;
    debug->arm_single_angle = s_arm.angle_last;
    debug->arm_turns = s_arm.turns;
    debug->arm_strike_cutoff = s_arm_strike_cutoff;
    debug->arm_current = s_arm_current_cmd;
    debug->toss_pos = s_toss.pos_abs;
    debug->toss_target = s_toss.target;
    debug->toss_single_angle = s_toss.angle_last;
    debug->toss_turns = s_toss.turns;
    debug->toss_current = s_toss_current_cmd;
    debug->magnet_on = s_magnet_on;
    debug->serve_current_on = s_serve_current_on;
    debug->magnet_pin_output = s_magnet_pin_output;
    debug->magnet_pin_cmd = s_magnet_pin_cmd;
    debug->magnet_pin_level = s_magnet_pin_level;
    debug->photo_pin_level = s_photo_pin_level;
    debug->photo_active = s_photo_active;
    debug->photo_trigger_count = s_photo_trigger_count;
    debug->photo_strike_pending = s_photo_strike_pending;
    debug->photo_strike_fired = s_photo_strike_fired;
}
