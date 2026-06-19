/**
  ******************************************************************************
  * @file    robot_control.c
  * @brief   主控制链、状态机、遥测和外部触发处理
  *
  *   本模块的控制逻辑从原 main.c 的 while(1) / app_scheduler 迁移而来，
  *   行为与重构前逐行等价；迁移期间做过的修正：type==2 空指针保护、
  *   拨杆 s 比较 | -> ||。robot_control_tick() 每个 1kHz 节拍执行一次完整控制链。
  ******************************************************************************
  */
#include "robot_control.h"

#include "main.h"
#include "bsp_can.h"
#include "pid.h"
#include "scheduler.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "BMI088driver.h"
#include "led_indicator.h"
#include "imu_filter.h"
#include "host_comm.h"
#include "chassis.h"
#include "dm_motor.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TELEM_PERIOD_MS        20U       // 遥测发送周期(ms)，20=50Hz
#define RC_CH_MAX              660.0f    // 遥控通道归一化满量程
#define RC_DEADZONE            0.05f     // 摇杆死区(归一化)，5%
#define REMOTE_AUTO_GAIN       5.0f      // 自动模式下遥控叠加增益
#define HOST_COORD_SCALE       100.0f    // 上位机坐标缩放(像素 -> 速度)

// 遥控四通道归一化结果（已套死区）
typedef struct {
    float x;   // 左右平移(Vy)
    float y;   // 前后(Vx)
    float z;   // 旋转(Wz)
    float w;   // 速度倍率
} remote_input_t;

static const RC_ctrl_t *rc_ctrl_point;               // 遥控器数据指针
static char telemetry_buffer[256];                   // 遥测输出缓冲

static motor_pid_t wz_pid;                           // Wz 轴回正 PID

// 状态机：拨杆边沿检测 + 校准计时
static int control_mode = 3;          // 控制模式(原 type)：0=校准 1=遥控 2=自动 3=失能
static char rc_sw_last = 0;           // 上一拍右拨杆值，用于边沿触发
static uint32_t calib_start_stamp = 0; // 进入校准(mode 0)的起始时间戳(g_tick, ms)

// BMI088 变量
static fp32 gyro[3], accel[3], temp;
static float accel_x, accel_y, accel_z;                // 加速度计测量值
static float accel_x_true, accel_y_true, accel_z_true; // 卡尔曼滤波后真实加速度
static float accel_x_bias = 0.0f, accel_y_bias = 0.0f, accel_z_bias = 0.0f; // 加速度计零偏

static uint32_t previous_time_stamp = 0; // 上一拍时间戳
static uint32_t current_time_stamp = 0;  // 当前拍时间戳

// Wz 保持控制
static float current_angle = 0.0f;       // 当前方向角度(gyro z 积分)
static float current_angle_err = 0.0f;   // 零飘(校准期估计)
static const float target_wz = 0.0f;     // 目标 Wz，即 0(正对方向)

// 摇杆死区：|归一化值| < 5% 视为 0，消除摇杆中位抖动
static float apply_deadzone(float value)
{
    return (fabsf(value) < RC_DEADZONE) ? 0.0f : value;  // fabsf：单精度，避免热路径双精度提升
}

// 读取遥控四通道并归一化(/660)，各自套用死区。
// 遥控掉线(指针为 NULL)时返回 0，调用方据此跳过本拍底盘更新。
// [修复] 原 type==2 代码缺此空指针保护，丢遥控时会崩。
static int read_remote_input(remote_input_t *input)
{
    if (rc_ctrl_point == NULL) {
        return 0;
    }

    input->z = apply_deadzone(-((float)rc_ctrl_point->rc.ch[0]) / RC_CH_MAX); // 旋转(Wz)
    input->y = apply_deadzone( ((float)rc_ctrl_point->rc.ch[1]) / RC_CH_MAX); // 前后(Vx) 上下摇杆
    input->x = apply_deadzone(-((float)rc_ctrl_point->rc.ch[2]) / RC_CH_MAX); // 左右平移(Vy) 左右摇杆
    input->w = apply_deadzone( ((float)rc_ctrl_point->rc.ch[3]) / RC_CH_MAX); // 速度倍率

    return 1;
}

// DM4340 执行机构(使能/控制环/发球)已抽至 application/dm_motor.c

// IMU 读取（BMI088，SPI1）
static void imu_update(void)
{
    BMI088_read(gyro, accel, &temp);
}

// 遥控器读取 + 模式选择（右拨杆 s[0]，边沿触发）
static void remote_update(void)
{
    rc_ctrl_point = get_remote_control_point();
    if (rc_ctrl_point != NULL) {
        char s = rc_ctrl_point->rc.s[0];
        // 仅在拨杆位置发生变化时设置 mode，避免每拍重置打断状态机内部转换
        // (原代码每拍无条件设 type，与 0->2 自动转换冲突，导致校准永远重启)
        if (s != rc_sw_last) {
            if (s == '\003' || s == '\000') {
                control_mode = 3;                  // 中档/初始 失能
            } else if (s == '\001') {
                control_mode = 1;                  // 上拨 遥控控制
            } else if (s == '\002') {
                control_mode = 0;                  // 下拨 进入校准
                calib_start_stamp = current_time_stamp;
            }
            rc_sw_last = s;
        }
    }
}

// mode 0：IMU 零偏校准。进入后第 1~2 秒积分 gyro z，2 秒末算零飘并转 mode 2
static void calibration_update(void)
{
    uint32_t elapsed = current_time_stamp - calib_start_stamp;  // 相对进入校准的时间

    if (elapsed > 1000U) {        // 进入 1 秒后开始积分(等待静止稳定)
        current_angle += gyro[2] * (current_time_stamp - previous_time_stamp);
    }

    if (elapsed > 2000U) {        // 2 秒末：取零飘、清零、转自动
        current_angle_err = current_angle / 1000.0f;
        current_angle = 0.0f;
        control_mode = 2;
    }
}

// mode 1：纯遥控模式
static void manual_chassis_update(void)
{
    remote_input_t input;

    if (read_remote_input(&input)) {
        chassis_set_velocity(input.y, input.x, input.z, input.w + 1.0f);
    }
}

// mode 2：自动追球 + 遥控融合（Wz 回正补偿）
static void auto_chassis_update(void)
{
    remote_input_t input;

    if (read_remote_input(&input)) {
        float dt = (float)(current_time_stamp - previous_time_stamp);
        int32_t host_x = uart_x;   // 上位机下发坐标(前后)
        int32_t host_y = uart_y;   // 上位机下发坐标(左右)

        // Wz 回正控制：积分当前角(扣除零飘)，PID 把方向拉回 0
        current_angle += (gyro[2] * dt) - (dt * current_angle_err);
        float wz_compensation = pid_calc(&wz_pid, target_wz, current_angle);

        // 上位机坐标 + 遥控融合
        float vx = -((float)host_x / HOST_COORD_SCALE) + input.y * REMOTE_AUTO_GAIN;
        float vy = input.x * REMOTE_AUTO_GAIN + wz_compensation;
        float wz = -((float)host_y / HOST_COORD_SCALE) + input.z * REMOTE_AUTO_GAIN;

        chassis_set_velocity(vx, vy, wz, input.w + 1.0f);
    }
}

// 状态机：按 control_mode 分发，结果经 chassis_set_velocity 写入底盘目标
static void state_machine_update(void)
{
    if (control_mode == 0) {
        calibration_update();      // 校准
    }

    if (control_mode == 1) {
        manual_chassis_update();   // 遥控
    }

    if (control_mode == 2) {
        auto_chassis_update();     // 自动
    }

    if (control_mode == 3) {
        chassis_stop();            // 失能
    }
}

// 加速度计卡尔曼滤波(逐轴独立，详见 imu_filter.c)
static void imu_kalman_update(void)
{
    accel_x = accel[0];
    accel_y = accel[1];
    accel_z = accel[2];

    kalman_filter_accel(accel_x, &accel_x_true, &accel_x_bias, 0);
    kalman_filter_accel(accel_y, &accel_y_true, &accel_y_bias, 1);
    kalman_filter_accel(accel_z, &accel_z_true, &accel_z_bias, 2);
}

// 遥测发送（频率由 robot_control_tick 分频控制，已移出控制热路径）
static void telemetry_update(void)
{
    s_motor_data_t *motor = &DM4340_Date[0];
    snprintf(telemetry_buffer, sizeof(telemetry_buffer), "%f,%f,%f,%f\r\n",
             motor->esc_back_position, motor->f_p, motor->esc_back_speed, motor->out_current);
    uart_queue_send(telemetry_buffer, strlen(telemetry_buffer));
}

// 后台任务：LED 状态指示 + UART 发送队列
static void housekeeping(void)
{
    // RGB LED 状态指示（蓝=初始化/绿闪=运行/红绿码=错误），非阻塞由节拍驱动
    led_update(current_time_stamp);

    // 处理 UART 发送队列（内部自检 DMA 空闲后再发）
    process_uart_queue();
}

// 应用层初始化：通信、传感器、CAN、遥控、各路 PID
void robot_control_init(void)
{
    host_comm_init();                   // 启动上位机 UART(USART1 单字节接收)

    // 时间刻初始化
    current_time_stamp = HAL_GetTick();
    previous_time_stamp = current_time_stamp;

    led_init();                         // LED 初始化(熄灭)
    led_set_init();                     // 初始化态
    led_update(0);                      // 立即点亮蓝灯(此时调度器尚未启动)

    while (BMI088_init()) {             // BMI088 初始化(失败则阻塞重试)
    }

    can_filter_init();                  // CAN 过滤器初始化
    remote_control_init();              // 遥控器初始化(USART3 DMA + 空闲中断)

    chassis_init();                     // 四路底盘速度环 PID 初始化
    dm_motor_init();                    // 三路 DM4340 执行电机控制初始化

    pid_init(&wz_pid, 1.5f, 0.00f, 0.0f, 500.0f, 660.0f);       // Wz 正对方向 PID
}

// 初始化完成，转正常运行态(LED 绿色闪烁)
void robot_control_start(void)
{
    led_set_running();
}

/* ============================================================================
 * 主控制节拍：由 TIM6 1kHz 节拍驱动(见 main 的 while(1))。
 * 控制链每拍执行(dt 恒为 1ms)；低频任务按节拍分频 + 相位错开调用。
 * ==========================================================================*/
void robot_control_tick(uint32_t tick_ms)
{
    previous_time_stamp = current_time_stamp;
    current_time_stamp = tick_ms;

    /* ---- 1kHz：完整控制链(与重构前等价，dt 恒 1ms) ---- */
    dm_motor_update();        // DM4340 使能 + 控制环 + 发球请求
    imu_update();             // BMI088 读取
    remote_update();          // 遥控器 + 模式选择
    state_machine_update();   // 状态机 + 麦轮解算
    chassis_update();         // 底盘速度环 + 电流下发
    imu_kalman_update();      // 加速度卡尔曼

    /* ---- 低频任务：模运算分频 + 相位错开(避开 ==0 拥挤) ---- */
    if (tick_ms % TELEM_PERIOD_MS == 7U) {   // 50Hz 遥测
        telemetry_update();
    }

    sched_tick(tick_ms);      // 扫描到期的延时事件(击球时序等)
    housekeeping();           // LED 心跳 + UART 队列
}

// PB12 外部中断处理(由 main 的 HAL_GPIO_EXTI_Callback 转发)
void robot_control_handle_exti(uint16_t gpio_pin)
{
    if (gpio_pin != GPIO_PIN_12) {
        return;
    }

    static uint32_t last_interrupt_time = 0;
    uint32_t now = HAL_GetTick();
    char temp_buffer[64];

    if (now - last_interrupt_time <= 50U) {   // 消抖处理(50ms)
        return;
    }
    // 注：PH10(蓝灯)现由 led_indicator 模块统一管理，此处不再翻转

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) {
        // 上升沿触发：仅记录
        snprintf(temp_buffer, sizeof(temp_buffer),
                 "[INT] PB12 Rising Edge at %lums\r\n", now);
    } else {
        // 下降沿触发：请求一次发球。仅置标志，设角度 + 定时回收在主循环上下文执行
        // (消除原先从 EXTI ISR 直接写 DM target_angle 的跨上下文访问)
        dm_motor_request_strike();

        snprintf(temp_buffer, sizeof(temp_buffer),
                 "[INT] PB12 Falling Edge at %lums\r\n", now);
    }

    uart_queue_send(temp_buffer, strlen(temp_buffer));
    last_interrupt_time = now;
}
