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
int16_t led_cnt;  // LED计数器
extern motor_measure_t motor_chassis; // 外部定义的电机信息结构体

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

double msp(double x, double in_min, double in_max, double out_min, double out_max)//映射函数，将编码器的值（0~8191）转换为弧度制的角度值（-pi~pi）
{
    return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

// 状态机s
int type=3;


// BMI088 变量定义
fp32 gyro[3], accel[3], temp;


// 全局变量
float accel_x, accel_y, accel_z; // 加速度计测量值
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
    // 时间刻初始化
    current_time_stamp = HAL_GetTick();
    previous_time_stamp  = HAL_GetTick();

    while (BMI088_init());              // BMI088 初始化
    can_filter_init();                  // can初始化
    remote_control_init();              // 遥控器初始化
    const RC_ctrl_t *rc_ctrl_point;     // 声明遥控器数据指针
    char uart_buffer[256];              // 定义 UART 输出缓冲区

    // 目标速度
    float target_speed1 = 0.0f; // rpm
    float target_speed2 = 0.0f; // rpm
    float target_speed3 = 0.0f; // rpm
    float target_speed4 = 0.0f; // rpm

    // 初始化四个底盘电机的 PID 控制器
    motor_pid_t motor1_pid, motor2_pid, motor3_pid, motor4_pid;
    pid_init(&motor1_pid, 3.5, 0.01, 0.0, 30000, 16384); // 调整参数
    pid_init(&motor2_pid, 3.5, 0.01, 0.0, 30000, 16384); // 调整参数
    pid_init(&motor3_pid, 3.5, 0.01, 0.0, 30000, 16384); // 调整参数
    pid_init(&motor4_pid, 3.5, 0.01, 0.0, 30000, 16384); // 调整参数

    // Wz 正对方向pid控制
    pid_init(&wz_pid, 1.5, 0.00, 0.0, 500, 660);






  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 初始化达妙电机使能
     CAN_cmd_motor_control(0x1,0x100,true);
     CAN_cmd_motor_control(0x2,0x100,true);
     CAN_cmd_motor_control(0x3,0x100,true);



      // 时间刻记录
      current_time_stamp = HAL_GetTick();

      // BMI088读取
      BMI088_read(gyro, accel, &temp);

      // 获取遥控器数据指针
      rc_ctrl_point = get_remote_control_point();

      if (rc_ctrl_point != NULL){
          // 遥控右拨杆控制
          char s = rc_ctrl_point->rc.s[0];
          if (s == '\003' | s == '\000'){type=3;}       // 初始0位中档3位 失能
          if (s == '\001'){type=1;}                     // 上拨1位 遥控控制
          if (s == '\002'){type=0;}                     // 下拨2位 角度保持
      }

      // 初始化
      if (type == 0){
          if (current_time_stamp>2000){
              current_angle_err = current_angle /1000;
              current_angle = 0;
              type = 2;
          }
          if (current_time_stamp>1000){
              current_angle += gyro[2] * ( current_time_stamp - previous_time_stamp );            // 积分得到当前角度 (gyro[2] 是 Z 轴角速度)
          }
      }

      if (type == 1){
          if (rc_ctrl_point != NULL){
              // 获取摇杆数据
              float z = -((float) rc_ctrl_point->rc.ch[0]); // 左右摇杆，右为正
              float y = (float) rc_ctrl_point->rc.ch[1]; // 上下摇杆，上为正
              float x = -((float) rc_ctrl_point->rc.ch[2]); // 旋转
              float w = (float) rc_ctrl_point->rc.ch[3];  // 速度

              // 映射摇杆数据到 -1 到 1 的范围
              float x_normalized = x / 660.0f;
              float y_normalized = y / 660.0f;
              float z_normalized = z / 660.0f;
              float w_normalized = w / 660.0f;


              // 定义底盘速度变量
              float Vx = 0.0f;
              float Vy = 0.0f;
              float Wz = 0.0f;


              // 添加死区
              float deadzone = 0.05f;
              if (fabs(x_normalized) < deadzone) {
                  x_normalized = 0.0f;
              }
              if (fabs(y_normalized) < deadzone) {
                  y_normalized = 0.0f;
              }
              if (fabs(z_normalized) < deadzone) {
                  z_normalized = 0.0f;
              }
              if (fabs(w_normalized) < deadzone) {
                  w_normalized = 0.0f;
              }



              // 根据摇杆数据计算底盘速度
              Vx = y_normalized;  // 上下摇杆控制前进/后退，上为正，前进
              Vy = x_normalized;  // 左右摇杆控制左右移动，右为正，左移
              Wz = z_normalized;  // 另一个摇杆控制旋转，正为逆时针



              // 根据底盘速度计算麦克纳姆轮速度
              target_speed1 = -(Vx + Vy + Wz)*1000*(w_normalized+1); // 右前方轮
              target_speed2 = (Vx - Vy - Wz)*1000*(w_normalized+1); // 左前方轮
              target_speed3 = -(Vx + Vy - Wz)*1000*(w_normalized+1); // 右后方轮
              target_speed4 = (Vx - Vy + Wz)*1000*(w_normalized+1); // 左后方轮


          }
      }
      // 状态二底盘Wz维持
      if (type == 2){
          // 定义底盘速度变量
          float Vx = 0.0f;
          float Vy = 0.0f;
          float Wz = 0.0f;


          // Wz 控制
          float dt = current_time_stamp - previous_time_stamp;
          current_angle += (gyro[2] * dt) - (dt * current_angle_err);            // 积分得到当前角度 (gyro[2] 是 Z 轴角速度)

          float wz_compensation = pid_calc(&wz_pid, target_wz, current_angle);// 计算 PID 控制器输出，用于补偿 Wz
          Wz = wz_compensation;

          // 根据底盘速度计算麦克纳姆轮速度
          target_speed1 = -(Vx + Vy + Wz)*1000; // 右前方轮
          target_speed2 = (Vx - Vy - Wz)*1000; // 左前方轮
          target_speed3 = -(Vx + Vy - Wz)*1000; // 右后方轮
          target_speed4 = (Vx - Vy + Wz)*1000; // 左后方轮

      }
      if (type == 3){
          target_speed1 = 0; // 右前方轮
          target_speed2 = 0; // 左前方轮
          target_speed3 = 0; // 右后方轮
          target_speed4 = 0; // 左后方轮
      }


      // 获取四个电机的速度
      const motor_measure_t *motor1 = get_chassis_motor_measure_point(0);
      const motor_measure_t *motor2 = get_chassis_motor_measure_point(1);
      const motor_measure_t *motor3 = get_chassis_motor_measure_point(2);
      const motor_measure_t *motor4 = get_chassis_motor_measure_point(3);


      // PID 计算
      float output1 = pid_calc(&motor1_pid, target_speed1, motor1->speed_rpm);
      float output2 = pid_calc(&motor2_pid, target_speed2, motor2->speed_rpm);
      float output3 = pid_calc(&motor3_pid, target_speed3, motor3->speed_rpm);
      float output4 = pid_calc(&motor4_pid, target_speed4, motor4->speed_rpm);

      // 电流控制 (将 PID 输出转换为 int16_t 类型)
      CAN_cmd_chassis((int16_t)output1, (int16_t)output2, (int16_t)output3, (int16_t)output4);
//      CAN_cmd_chassis1((int16_t)output1,(int16_t)output2);
//      CAN_cmd_chassis2((int16_t)output3, (int16_t)output4);





      accel_x = accel[0];
      accel_y = accel[1];
      accel_z = accel[2];


      // 卡尔曼滤波
      kalman_filter_accel(accel_x, &accel_x_true, &accel_x_bias, 0);
      kalman_filter_accel(accel_y, &accel_y_true, &accel_y_bias, 1);
      kalman_filter_accel(accel_z, &accel_z_true, &accel_z_bias, 2);



      led_cnt ++;// LED计数器自增
      if (led_cnt == 2)
      {
          led_cnt = 0;
          HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11); //blink cycle 500ms
          // 翻转GPIOH, PIN11引脚的电平，实现LED闪烁，周期为500ms (250 * 2ms)


          if(dma_tx_complete == 1){

              // 电机速度格式化
              sprintf(uart_buffer, "%d,%d,%d,%d\r\n",
                      motor1->speed_rpm, motor2->speed_rpm, motor3->speed_rpm, motor4->speed_rpm);


              // IMU数值格式化
//              sprintf(uart_buffer, "Gyro: %.11f, %.11f, %.11f  Accel: %.11f, %.11f, %.11f\r\n",
//                      gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);


//              sprintf(uart_buffer, "Gyro: %.11f, %.11f, %.11f, %.11f\r\n",
//                      gyro[0], gyro[1], gyro[2],current_angle);


              // 原始数据
//              sprintf(uart_buffer, "Accel: %.11f, %.11f, %.11f\r\n",
//                      accel[0], accel[1], accel[2]);
              // 卡尔曼滤波
//              sprintf(uart_buffer, "Accel: %.11f, %.11f, %.11f\r\n",
//                      accel_x_true, accel_y_true, accel_z_true);




          process_uart_queue();
          }
     }






//      HAL_Delay(40);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      previous_time_stamp = current_time_stamp;
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

    // 先将时钟源选择为内部时钟
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
                CAN_cmd_motor_pos_vel_control(0,0.5);


                snprintf(temp_buffer, sizeof(temp_buffer), 
                        "[INT] PB12 Rising Edge at %lums\r\n", current_time);
            } else {
                // 下降沿触发
               CAN_cmd_motor_pos_vel_control(0.5,0.5);


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

#ifdef  USE_FULL_ASSERT
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
