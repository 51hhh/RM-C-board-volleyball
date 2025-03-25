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

int16_t led_cnt;  // LED计数器
extern motor_measure_t motor_chassis; // 外部定义的电机信息结构体
extern pid_struct_t gimbal_yaw_speed_pid; // 外部定义的速度环PID结构体


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
  /* USER CODE BEGIN 2 */
    can_filter_init();//can初始化
    gimbal_PID_init();//PID初始化
    remote_control_init();//遥控器初始化
    const RC_ctrl_t *rc_ctrl_point; // 声明遥控器数据指针
    char uart_buffer[256];  // 定义 UART 输出缓冲区

    // 初始化四个底盘电机的 PID 控制器
    pid_struct_t motor1_pid, motor2_pid, motor3_pid, motor4_pid;
    pid_init(&motor1_pid, 1.5, 0, 0, 30000, 16384); // 调整参数
    pid_init(&motor2_pid, 1, 0, 0, 30000, 16384); // 调整参数
    pid_init(&motor3_pid, 1, 0, 0, 30000, 16384); // 调整参数
    pid_init(&motor4_pid, 1, 0, 0, 30000, 16384); // 调整参数

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


//      CAN_cmd_chassis(-1000, 1000, -1000, 1000);

      // 获取四个电机的速度
      const motor_measure_t *motor1 = get_chassis_motor_measure_point(0);
      const motor_measure_t *motor2 = get_chassis_motor_measure_point(1);
      const motor_measure_t *motor3 = get_chassis_motor_measure_point(2);
      const motor_measure_t *motor4 = get_chassis_motor_measure_point(3);

      // 设定目标速度 (这里可以根据你的需求修改)
      float target_speed1 = 1000.0f; // rpm
      float target_speed2 = -1000.0f; // rpm
      float target_speed3 = 1000.0f; // rpm
      float target_speed4 = -1000.0f; // rpm


      // PID 计算
      float output1 = pid_calc(&motor1_pid, target_speed1, motor1->speed_rpm);
      float output2 = pid_calc(&motor2_pid, target_speed2, motor2->speed_rpm);
      float output3 = pid_calc(&motor3_pid, target_speed3, motor3->speed_rpm);
      float output4 = pid_calc(&motor4_pid, target_speed4, motor4->speed_rpm);

      // 电流控制 (将 PID 输出转换为 int16_t 类型)
//      CAN_cmd_chassis((int16_t)output1, (int16_t)output2, (int16_t)output3, (int16_t)output4);
      CAN_cmd_chassis((int16_t)output1,(int16_t)output2,0,0);


      led_cnt ++;// LED计数器自增
      if (led_cnt == 25)
      {
          led_cnt = 0;
          HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11); //blink cycle 500ms
          // 翻转GPIOH, PIN11引脚的电平，实现LED闪烁，周期为500ms (250 * 2ms)
      }


//          // 格式化输出电机速度到 UART1
//          sprintf(uart_buffer, "M1:%d, M2:%d, M3:%d, M4:%d\r\n",
//                  motor1->speed_rpm, motor2->speed_rpm, motor3->speed_rpm, motor4->speed_rpm);
//
//          // 通过 UART1 发送数据
//          HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);





//      // 获取遥控器数据指针
//      rc_ctrl_point = get_remote_control_point();
//
//
//      if (rc_ctrl_point != NULL)
//      {
//          // 使用遥控器数据计算目标角度
//          // 假设 ch[0] 和 ch[1] 的范围是 -660 到 660
//          float x = (float)rc_ctrl_point->rc.ch[0];
//          float y = (float)rc_ctrl_point->rc.ch[1];
//
//          // 映射 x 和 y 到 -1 到 1 的范围
//          float x_normalized = x / 660.0f;
//          float y_normalized = y / 660.0f;
//
//          // 使用 atan2 计算角度
//          target_yaw_angle = atan2f(y_normalized, x_normalized);
//
//          // 限制角度在 -PI 到 PI 之间
//          if (target_yaw_angle > M_PI) {
//              target_yaw_angle -= 2 * M_PI;
//          } else if (target_yaw_angle < -M_PI) {
//              target_yaw_angle += 2 * M_PI;
//          }
//      }
//      now_yaw_angle=msp(motor_yaw_info.rotor_angle,0,8191,-M_PI,M_PI);//计算当前的编码器角度值，运用msp函数将编码器的值映射为弧度制
//      //计算当前的编码器角度值，运用msp函数将编码器的值映射为弧度制，范围从0-8191映射到-pi到pi
//      pid_calc(&gimbal_yaw_angle_pid,target_yaw_angle, now_yaw_angle);
//      //角度环PID计算，输入目标角度和当前角度，计算输出
//      pid_calc(&gimbal_yaw_speed_pid,gimbal_yaw_angle_pid.output, motor_yaw_info.rotor_speed);//速度环
//      //速度环PID计算，输入目标速度（角度环的输出）和当前速度，计算输出
//      set_GM6020_motor_voltage(&hcan1,gimbal_yaw_speed_pid.output);
//      //can发送函数，发送经过PID计算的电压值，控制GM6020电机


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
