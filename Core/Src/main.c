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


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//自定义私有类型定义 (typedef)

uint8_t dma_tx_complete = 1;  // DMA 传输完成标志位
int16_t led_cnt;  // LED计数器
extern motor_measure_t motor_chassis; // 外部定义的电机信息结构体



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
  /* USER CODE BEGIN 2 */
    can_filter_init();//can初始化
    remote_control_init();//遥控器初始化
    const RC_ctrl_t *rc_ctrl_point; // 声明遥控器数据指针
    char uart_buffer[256];  // 定义 UART 输出缓冲区

    // 目标速度
    float target_speed1 = 0.0f; // rpm
    float target_speed2 = 0.0f; // rpm
    float target_speed3 = 0.0f; // rpm
    float target_speed4 = 0.0f; // rpm

    // 初始化四个底盘电机的 PID 控制器
    motor_pid_t motor1_pid, motor2_pid, motor3_pid, motor4_pid;
    pid_init(&motor1_pid, 4.5, 0.01, 0.0, 30000, 16384); // 调整参数
    pid_init(&motor2_pid, 4.5, 0.01, 0.0, 30000, 16384); // 调整参数
    pid_init(&motor3_pid, 3.5, 0.01, 0.0, 30000, 16384); // 调整参数
    pid_init(&motor4_pid, 3.5, 0.01, 0.0, 30000, 16384); // 调整参数

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 获取遥控器数据指针
      rc_ctrl_point = get_remote_control_point();
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
//      CAN_cmd_chassis((int16_t)output1, (int16_t)output2, (int16_t)output3, (int16_t)output4);
      CAN_cmd_chassis1((int16_t)output1,(int16_t)output2);
      CAN_cmd_chassis2((int16_t)output3, (int16_t)output4);


      led_cnt ++;// LED计数器自增
      if (led_cnt == 2)
      {
          led_cnt = 0;
          HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11); //blink cycle 500ms
          // 翻转GPIOH, PIN11引脚的电平，实现LED闪烁，周期为500ms (250 * 2ms)


          if(dma_tx_complete == 1){
              // 格式化输出电机速度到 UART1
              sprintf(uart_buffer, "%d,%d,%d,%d\r\n",
                      motor1->speed_rpm, motor2->speed_rpm, motor3->speed_rpm, motor4->speed_rpm);

              // 通过 UART1 发送数据 (使用 DMA)
              dma_tx_complete = 0;  // 重置 DMA 传输完成标志位
              HAL_UART_Transmit_DMA(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer));
          }
     }






      HAL_Delay(40);

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
