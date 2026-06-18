/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// 自定义头文件
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "bsp_can.h"
#include "pid.h"
#include "../application/CAN_receive.h"
#include "../application/remote_control.h"
#include "../application/BMI088driver.h"
#include "scheduler.h"   // 1kHz 时基 + 延时事件调度

// 位置环PID控制器
motor_pid_t pos_x_pid, pos_y_pid;


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//自定义私有类型定义 (typedef)

#define UART_QUEUE_SIZE 32

typedef struct {
    char buffer[128];
    uint16_t length;
} uart_queue_item_t;

uart_queue_item_t uart_queue[UART_QUEUE_SIZE];
uint16_t uart_queue_head = 0;
uint16_t uart_queue_tail = 0;
uint8_t dma_tx_complete = 1;  // DMA 传输完成标志位
uint32_t led_blink_stamp = 0;  // LED 心跳：上次翻转的时间戳(ms)

/* ===== 主循环模块化：共享控制状态（原为 main() 局部变量，提升为文件作用域）===== */
static const RC_ctrl_t *rc_ctrl_point;            // 遥控器数据指针
static char uart_buffer[256];                     // UART 输出缓冲区
static float target_speed1, target_speed2, target_speed3, target_speed4; // 麦轮目标转速(rpm)
static motor_pid_t motor1_pid, motor2_pid, motor3_pid, motor4_pid;       // 4 个底盘电机速度环

/* ===== 多频率调度参数 =====
 * 时基由 TIM6@1kHz 提供(见 scheduler.c)，主控制链每拍(1ms)执行；
 * 低频任务在 app_scheduler 中按节拍分频调用。
 * 注意：pid_calc 无 dt，依赖 dt 的控制任务必须每拍执行，不可降频。
 */
#define TELEM_PERIOD_MS   20    // 遥测发送周期(ms)，20=50Hz
extern motor_measure_t motor_chassis; // 外部定义的电机信息结构体

// uart 消息发送添加队列函数
void uart_queue_send(const char* data, uint16_t length) {
    if ((uart_queue_head + 1) % UART_QUEUE_SIZE != uart_queue_tail) {
        strncpy(uart_queue[uart_queue_head].buffer, data, length);
        uart_queue[uart_queue_head].length = length;
        uart_queue_head = (uart_queue_head + 1) % UART_QUEUE_SIZE;
    }
}

void process_uart_queue(void) {
    if (dma_tx_complete && uart_queue_tail != uart_queue_head) {
        dma_tx_complete = 0;
        HAL_UART_Transmit_DMA(&huart1, 
                            (uint8_t *)uart_queue[uart_queue_tail].buffer, 
                            uart_queue[uart_queue_tail].length);
        uart_queue_tail = (uart_queue_tail + 1) % UART_QUEUE_SIZE;
    }
}



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// UART1 接收缓冲区定义
#define UART1_RX_BUFFER_SIZE 128
uint8_t uart1_rx_buffer[UART1_RX_BUFFER_SIZE];
uint8_t uart1_rx_byte; // 单字节接收缓冲区
uint8_t uart1_line_buffer[128]; // 行缓冲区
uint16_t uart1_line_pos = 0; // 行缓冲区当前位置

double msp(double x, double in_min, double in_max, double out_min, double out_max)//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
{
    return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

// 状态机s
int type=3;


// BMI088 变量定义
fp32 gyro[3], accel[3], temp;

// 状态机：拨杆边沿检测 + type=0 校准计时
char     rc_sw_last = 0;          // 上一拍右拨杆值，用于边沿触发
uint32_t calib_start_stamp = 0;   // 进入 type=0 的起始时间戳(g_tick, ms)


// 全局变量
float accel_x, accel_y, accel_z; // 加速度计测量值
int32_t uart_x = 0;  // 从串口接收的X坐标
int32_t uart_y = 0;  // 从串口接收的Y坐标
float accel_x_true, accel_y_true, accel_z_true; // 真实加速度
float accel_x_bias = 0.0f, accel_y_bias = 0.0f, accel_z_bias = 0.0f; // 加速度计零偏
uint32_t previous_time_stamp = 0; // 上一次的时间戳
uint32_t current_time_stamp = 0;  // 当前的时间戳

// Wz 保持控制
float current_angle = 0.0f;   // 当前方向角度
float current_angle_err = 0.0f; // 零飘
motor_pid_t wz_pid;          // 用于 Wz 轴回正的 PID 控制器
float target_wz = 0.0f;     // 目标Wz值，即0

// 卡尔曼滤波参数 (需要根据实际情况调整)
float Q_accel = 0.001f; // 过程噪声 (加速度)
float Q_bias = 0.00001f; // 过程噪声 (零偏)
float R_accel_meas = 0.01f; // 测量噪声 (加速度计)

float P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}}; // 初始状态协方差矩阵

// 卡尔曼滤波更新 (单轴)
void kalman_filter_accel(float measured_accel, float *true_accel, float *bias, int axis) {
    // 1. 预测步骤
    float F[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}}; // 状态转移矩阵
    float x_hat[2] = {*true_accel, *bias}; // 状态向量

    // 预测状态
    float x_hat_predicted[2];
    x_hat_predicted[0] = F[0][0] * x_hat[0] + F[0][1] * x_hat[1];
    x_hat_predicted[1] = F[1][0] * x_hat[0] + F[1][1] * x_hat[1];

    // 预测协方差矩阵
    float P_predicted[2][2];
    P_predicted[0][0] = F[0][0] * P[0][0] * F[0][0] + F[0][1] * P[1][0] * F[0][0] + F[0][0] * P[0][1] * F[1][0] + F[0][1] * P[1][1] * F[1][0] + Q_accel;
    P_predicted[0][1] = F[0][0] * P[0][0] * F[0][1] + F[0][1] * P[1][0] * F[0][1] + F[0][0] * P[0][1] * F[1][1] + F[0][1] * P[1][1] * F[1][1];
    P_predicted[1][0] = F[1][0] * P[0][0] * F[0][0] + F[1][1] * P[1][0] * F[0][0] + F[1][0] * P[0][1] * F[1][0] + F[1][1] * P[1][1] * F[1][0];
    P_predicted[1][1] = F[1][0] * P[0][0] * F[0][1] + F[1][1] * P[1][0] * F[0][1] + F[1][0] * P[0][1] * F[1][1] + F[1][1] * P[1][1] * F[1][1] + Q_bias;

    // 2. 更新步骤
    float H[1][2] = {{1.0f, 1.0f}}; // 测量矩阵
    float z = measured_accel; // 测量值

    // 计算残差
    float y = z - (H[0][0] * x_hat_predicted[0] + H[0][1] * x_hat_predicted[1]);

    // 计算残差协方差
    float S = H[0][0] * P_predicted[0][0] * H[0][0] + H[0][1] * P_predicted[1][0] * H[0][0] + H[0][0] * P_predicted[0][1] * H[0][1] + H[0][1] * P_predicted[1][1] * H[0][1] + R_accel_meas;

    // 计算卡尔曼增益
    float K[2];
    K[0] = (P_predicted[0][0] * H[0][0] + P_predicted[0][1] * H[0][1]) / S;
    K[1] = (P_predicted[1][0] * H[0][0] + P_predicted[1][1] * H[0][1]) / S;

    // 更新状态估计
    *true_accel = x_hat_predicted[0] + K[0] * y;
    *bias = x_hat_predicted[1] + K[1] * y;

    // 更新协方差矩阵
    P[0][0] = (1 - K[0] * H[0][0]) * P_predicted[0][0] - K[0] * H[0][1] * P_predicted[1][0];
    P[0][1] = (1 - K[0] * H[0][0]) * P_predicted[0][1] - K[0] * H[0][1] * P_predicted[1][1];
    P[1][0] = -K[1] * H[0][0] * P_predicted[0][0] + (1 - K[1] * H[0][1]) * P_predicted[1][0];
    P[1][1] = -K[1] * H[0][0] * P_predicted[0][1] + (1 - K[1] * H[0][1]) * P_predicted[1][1];
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================================
 * 主循环模块化：以下函数从原 while(1) 循环体拆分而来，控制逻辑保持不变。
 * 顺序与原循环一致；仅修复 type==2 空指针解引用、s 比较的 | -> ||。
 * ==========================================================================*/

// DM4340 执行电机：使能 + PID 控制
static void dm4340_update(void)
{
    CAN_cmd_motor_control(0x1, 0, true);
    CAN_cmd_motor_control(0x2, 0, true);
    CAN_cmd_motor_control(0x3, 0, true);

    DM4340_Control_Loop(0);
    DM4340_Control_Loop(1);
    DM4340_Control_Loop(2);
}

// IMU 读取（BMI088）
static void imu_update(void)
{
    BMI088_read(gyro, accel, &temp);
}

// 遥控器读取 + 模式选择（右拨杆，边沿触发）
static void remote_update(void)
{
    rc_ctrl_point = get_remote_control_point();
    if (rc_ctrl_point != NULL) {
        char s = rc_ctrl_point->rc.s[0];
        // 仅在拨杆位置发生变化时设置 type，避免每拍重置打断状态机内部转换
        // (原代码每拍无条件设 type，与 type 0→2 自动转换冲突，导致校准永远重启)
        if (s != rc_sw_last) {
            if (s == '\003' || s == '\000') { type = 3; }      // 中档/初始 失能
            else if (s == '\001') { type = 1; }                // 上拨 遥控控制
            else if (s == '\002') { type = 0; calib_start_stamp = current_time_stamp; } // 下拨 进入校准
            rc_sw_last = s;
        }
    }
}

// 状态机 + 麦克纳姆轮解算 -> target_speed1..4
static void state_machine_update(void)
{
    // type 0：IMU 零偏校准，进入后第 1~2 秒积分，2 秒末算零偏并转 type 2
    if (type == 0) {
        uint32_t elapsed = current_time_stamp - calib_start_stamp;  // 相对进入校准的时间
        if (elapsed > 1000) {
            current_angle += gyro[2] * (current_time_stamp - previous_time_stamp);
        }
        if (elapsed > 2000) {
            current_angle_err = current_angle / 1000;
            current_angle = 0;
            type = 2;
        }
    }

    // type 1：纯遥控模式
    if (type == 1) {
        if (rc_ctrl_point != NULL) {
            float z = -((float) rc_ctrl_point->rc.ch[0]);
            float y = (float) rc_ctrl_point->rc.ch[1];
            float x = -((float) rc_ctrl_point->rc.ch[2]);
            float w = (float) rc_ctrl_point->rc.ch[3];

            float x_normalized = x / 660.0f;
            float y_normalized = y / 660.0f;
            float z_normalized = z / 660.0f;
            float w_normalized = w / 660.0f;

            float Vx = 0.0f;
            float Vy = 0.0f;
            float Wz = 0.0f;

            float deadzone = 0.05f;
            if (fabs(x_normalized) < deadzone) { x_normalized = 0.0f; }
            if (fabs(y_normalized) < deadzone) { y_normalized = 0.0f; }
            if (fabs(z_normalized) < deadzone) { z_normalized = 0.0f; }
            if (fabs(w_normalized) < deadzone) { w_normalized = 0.0f; }

            Vx = y_normalized;  // 上下摇杆：前进/后退
            Vy = x_normalized;  // 左右摇杆：左右平移
            Wz = z_normalized;  // 旋转

            target_speed1 = -(Vx + Vy + Wz) * 1000 * (w_normalized + 1); // 右前
            target_speed2 =  (Vx - Vy - Wz) * 1000 * (w_normalized + 1); // 左前
            target_speed3 = -(Vx + Vy - Wz) * 1000 * (w_normalized + 1); // 右后
            target_speed4 =  (Vx - Vy + Wz) * 1000 * (w_normalized + 1); // 左后
        }
    }

    // type 2：自动追球 + 遥控融合（Wz 回正补偿）
    if (type == 2) {
        if (rc_ctrl_point != NULL) {   // [修复] 原代码缺少空指针保护，丢遥控时会崩
            float z = -((float) rc_ctrl_point->rc.ch[0]);
            float y = (float) rc_ctrl_point->rc.ch[1];
            float x = -((float) rc_ctrl_point->rc.ch[2]);
            float w = (float) rc_ctrl_point->rc.ch[3];

            float x_normalized = x / 660.0f;
            float y_normalized = y / 660.0f;
            float z_normalized = z / 660.0f;
            float w_normalized = w / 660.0f;

            float deadzone = 0.05f;
            if (fabs(x_normalized) < deadzone) { x_normalized = 0.0f; }
            if (fabs(y_normalized) < deadzone) { y_normalized = 0.0f; }
            if (fabs(z_normalized) < deadzone) { z_normalized = 0.0f; }
            if (fabs(w_normalized) < deadzone) { w_normalized = 0.0f; }

            // Wz 回正控制
            float dt = current_time_stamp - previous_time_stamp;
            current_angle += (gyro[2] * dt) - (dt * current_angle_err);
            float wz_compensation = pid_calc(&wz_pid, target_wz, current_angle);

            // 上位机坐标 + 遥控融合
            float Vx = -(uart_x / 100.0f) + y_normalized * 5;
            float Vy = x_normalized * 5 + wz_compensation;
            float Wz = -(uart_y / 100.0f) + z_normalized * 5;

            target_speed1 = -(Vx + Vy + Wz) * 1000 * (w_normalized + 1); // 右前
            target_speed2 =  (Vx - Vy - Wz) * 1000 * (w_normalized + 1); // 左前
            target_speed3 = -(Vx + Vy - Wz) * 1000 * (w_normalized + 1); // 右后
            target_speed4 =  (Vx - Vy + Wz) * 1000 * (w_normalized + 1); // 左后
        }
    }

    // type 3：失能
    if (type == 3) {
        target_speed1 = 0;
        target_speed2 = 0;
        target_speed3 = 0;
        target_speed4 = 0;
    }
}

// 底盘速度环 PID + 电流下发
static void chassis_pid_update(void)
{
    const motor_measure_t *motor1 = get_chassis_motor_measure_point(0);
    const motor_measure_t *motor2 = get_chassis_motor_measure_point(1);
    const motor_measure_t *motor3 = get_chassis_motor_measure_point(2);
    const motor_measure_t *motor4 = get_chassis_motor_measure_point(3);

    float output1 = pid_calc(&motor1_pid, target_speed1, motor1->speed_rpm);
    float output2 = pid_calc(&motor2_pid, target_speed2, motor2->speed_rpm);
    float output3 = pid_calc(&motor3_pid, target_speed3, motor3->speed_rpm);
    float output4 = pid_calc(&motor4_pid, target_speed4, motor4->speed_rpm);

    CAN_cmd_chassis((int16_t)output1, (int16_t)output2, (int16_t)output3, (int16_t)output4);
}

// 加速度计卡尔曼滤波
static void imu_kalman_update(void)
{
    accel_x = accel[0];
    accel_y = accel[1];
    accel_z = accel[2];

    kalman_filter_accel(accel_x, &accel_x_true, &accel_x_bias, 0);
    kalman_filter_accel(accel_y, &accel_y_true, &accel_y_bias, 1);
    kalman_filter_accel(accel_z, &accel_z_true, &accel_z_bias, 2);
}

// EXTI 上升沿触发的延时任务（DM4340 设角度）
// 延时回调：EXTI 触发 500ms 后让 DM4340 回到 0.1 弧度
// （替代原 g_rising_edge_triggered 轮询，由事件调度器 schedule_after 调用）
static void cb_dm_recover(void)
{
    DM4340_Set_Target_Angle(0, 0.1);
    DM4340_Set_Target_Angle(1, 0.1);
    DM4340_Set_Target_Angle(2, 0.1);
}

// 遥测发送（调用频率由 app_scheduler 分频控制，已移出控制热路径）
static void telemetry_update(void)
{
    s_motor_data_t *motor = &DM4340_Date[0];
    sprintf(uart_buffer, "%f,%f,%f,%f\r\n",
            motor->esc_back_position, motor->f_p, motor->esc_back_speed, motor->out_current);
    uart_queue_send(uart_buffer, strlen(uart_buffer));
}

// 后台任务：LED 心跳 + UART 发送队列
static void housekeeping(void)
{
    // LED 心跳：基于系统时间每 500ms 翻转一次，与主循环速度解耦。
    if (current_time_stamp - led_blink_stamp >= 500) {
        led_blink_stamp = current_time_stamp;
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
    }

    // 处理 UART 发送队列（DMA 空闲时发送）
    if (dma_tx_complete == 1) {
        process_uart_queue();
    }
}

/* ============================================================================
 * 主调度器：由 TIM6 1kHz 节拍驱动（见 main 的 while(1)）。
 * 控制链每拍执行（dt 恒为 1ms）；低频任务按节拍分频 + 相位错开调用。
 * ==========================================================================*/
static void app_scheduler(uint32_t t)
{
    previous_time_stamp = current_time_stamp;
    current_time_stamp  = t;

    /* ---- 1kHz：完整控制链（与重构前等价，dt 恒 1ms）---- */
    dm4340_update();          // DM4340 使能 + PID
    imu_update();             // BMI088 读取
    remote_update();          // 遥控器 + 模式选择
    state_machine_update();   // 状态机 + 麦轮解算
    chassis_pid_update();     // 底盘速度环 + 电流下发
    imu_kalman_update();      // 加速度卡尔曼

    /* ---- 低频任务：模运算分频 + 相位错开 ----
     * 注意：依赖 dt 的控制任务不可放到低频槽。 */
    if (t % TELEM_PERIOD_MS == 7) telemetry_update();   // 50Hz 遥测
    // if (t % 10 == 3) some_100hz_task();              // 100Hz 任务示例

    /* ---- 延时事件 + 后台任务 ---- */
    sched_tick(t);            // 扫描到期的延时触发（击球时序等）
    housekeeping();           // LED 心跳 + UART 队列
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
    // 启动USART1单字节接收模式
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);

    // 时间刻初始化
    current_time_stamp = HAL_GetTick();
    previous_time_stamp  = HAL_GetTick();

    while (BMI088_init());              // BMI088 初始化
    can_filter_init();                  // can初始化
    remote_control_init();              // 遥控器初始化

    // 初始化四个底盘电机的 PID 控制器（变量已提升为文件作用域）
    pid_init(&motor1_pid, 3.5, 0.01, 0.0, 30000, 16384); // 调整参数
    pid_init(&motor2_pid, 3.5, 0.01, 0.0, 30000, 16384); // 调整参数
    pid_init(&motor3_pid, 3.5, 0.01, 0.0, 30000, 16384); // 调整参数
    pid_init(&motor4_pid, 3.5, 0.01, 0.0, 30000, 16384); // 调整参数

    // Wz 正对方向pid控制
    pid_init(&wz_pid, 1.5, 0.00, 0.0, 500, 660);

    // 位置环PID初始化
    pid_init(&pos_x_pid, 0.6f, 0.01f, 0.2f, 1000, 300); // X方向位置环
    pid_init(&pos_y_pid, 0.6f, 0.01f, 0.2f, 1000, 300); // Y方向位置环



    //初始化电机控制参数
    DM4340_Control_Init(0);  // ID 0x01电机PID控制器初始化
    DM4340_Control_Init(1);
    DM4340_Control_Init(2);

    timebase_init();         // 启动 TIM6@1kHz 时基 + DWT 微秒计时

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 由 TIM6 1kHz 中断驱动：中断里置 g_tick_flag，主循环消费一次即跑一拍调度。
      // 即使某拍处理超过 1ms 偶尔丢标志，g_tick 仍是最新值，dt 能反映真实流逝时间。
      if (g_tick_flag)
      {
          g_tick_flag = 0;
          app_scheduler(g_tick);
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // 先将时钟源选择为内部时钟（GDB调试时避免时钟跑飞）
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// DMA 传输完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        dma_tx_complete = 1;  // 设置 DMA 传输完成标志位
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) {
        // 检查是否收到换行符
        if(uart1_rx_byte == '\n') {
            // 完成一行接收，添加字符串结束符
            uart1_line_buffer[uart1_line_pos] = '\0';
            
            // 尝试解析"int,int"格式的坐标
            if(sscanf((char*)uart1_line_buffer, "%d,%d", &uart_y, &uart_x) == 2) {
                char temp_buffer[64];
                snprintf(temp_buffer, sizeof(temp_buffer), "Parsed coordinates: X=%d, Y=%d\r\n", 
                        uart_y, uart_x);
                uart_queue_send(temp_buffer, strlen(temp_buffer));
            }
            
            // 重置行缓冲区
            uart1_line_pos = 0;
        } else {
            // 存储接收到的字节
            if(uart1_line_pos < sizeof(uart1_line_buffer)-1) {
                uart1_line_buffer[uart1_line_pos++] = uart1_rx_byte;
            }
        }
        
        // 重新启动单字节接收
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
}

/* PB12中断回调函数 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_12) {
        static uint32_t last_interrupt_time = 0;
        uint32_t current_time = HAL_GetTick();
        char temp_buffer[64];
        
        // 消抖处理(50ms)
        if(current_time - last_interrupt_time > 50) {
            // 翻转LED作为调试指示
            HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
            
            if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) {
                // 上升沿触发
                // DM4340_Set_Target_Angle(0, 0);
                // DM4340_Set_Target_Angle(1, 0);
                // DM4340_Set_Target_Angle(2, 0);

                snprintf(temp_buffer, sizeof(temp_buffer), 
                        "[INT] PB12 Rising Edge at %lums\r\n", current_time);
            } else {
                // 下降沿触发：立即设角度 1.1，并在 500ms 后非阻塞回到 0.1
                DM4340_Set_Target_Angle(0, 1.1);
                DM4340_Set_Target_Angle(1, 1.1);
                DM4340_Set_Target_Angle(2, 1.1);

                schedule_after(500, cb_dm_recover); // 延时触发，替代原轮询标志位

                snprintf(temp_buffer, sizeof(temp_buffer),
                        "[INT] PB12 Falling Edge at %lums\r\n", current_time);
            }
            
            uart_queue_send(temp_buffer, strlen(temp_buffer));
            last_interrupt_time = current_time;
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
