# RM-C-board-volleyball 排球机器人底盘控制系统

基于 **STM32F407IGHx** 的排球机器人嵌入式控制系统，实现麦克纳姆轮全向底盘运动控制、DM4340 电机 MIT 模式控制、BMI088 IMU 姿态感知、SBUS 遥控器控制，并通过串口接收上位机视觉检测坐标实现自动追球。

## 特性

- **麦克纳姆轮全向底盘**：4 轮独立 PID 速度闭环，支持全向移动与旋转
- **DM4340 电机控制**：CAN 总线 MIT 控制模式，支持位置/速度/力矩控制
- **M3508 底盘电机**：CAN 通信，速度环 PID 控制
- **BMI088 六轴 IMU**：SPI 接口驱动，加速度 + 陀螺仪数据采集
- **卡尔曼滤波**：加速度计零偏估计与噪声滤波
- **陀螺仪航向保持**：Z 轴角速度积分 + PID 回正控制
- **SBUS 遥控器**：DMA 双缓冲接收，支持多档位状态切换
- **串口坐标接收**：接收上位机视觉系统发送的排球坐标，融合至底盘运动控制
- **3 种控制模式**：遥控模式 / 自动追球模式 / 失能模式
- **GPIO 中断触发**：外部信号触发 DM4340 电机动作序列

## 系统架构

```
┌──────────────┐  UART 115200bps  ┌────────────────────────────────────┐
│  上位机       │ ───────────────▶ │  STM32F407IGHx (168MHz)           │
│  视觉检测系统 │  球心坐标 x,y    │                                    │
│  (yolov8/     │                  │  ┌──────────┐   ┌──────────────┐  │
│   nanodet)    │                  │  │ SBUS遥控  │   │ BMI088 IMU   │  │
└──────────────┘                  │  │ (USART3)  │   │ (SPI1)       │  │
                                   │  └─────┬────┘   └──────┬───────┘  │
                                   │        │                │          │
                                   │        ▼                ▼          │
                                   │  ┌─────────────────────────────┐  │
                                   │  │    主控制循环               │  │
                                   │  │    状态机 + PID 控制        │  │
                                   │  │    麦克纳姆轮运动学解算     │  │
                                   │  │    卡尔曼滤波               │  │
                                   │  └─────────┬───────────────────┘  │
                                   │            │                      │
                                   │     ┌──────┴──────┐               │
                                   │     ▼             ▼               │
                                   │  ┌───────┐  ┌──────────┐         │
                                   │  │ CAN1  │  │  CAN2    │         │
                                   │  │M3508  │  │ DM4340   │         │
                                   │  │底盘x4 │  │ 执行x3   │         │
                                   │  └───────┘  └──────────┘         │
                                   └────────────────────────────────────┘
```

## 项目结构

```
RM-C-board-volleyball/
├── Core/
│   ├── Src/
│   │   ├── main.c                 # 主控制循环、状态机、麦轮解算、卡尔曼滤波
│   │   ├── pid.c                  # PID 控制器实现（带积分限幅）
│   │   └── ...                    # HAL 外设初始化
│   ├── Inc/
│   │   ├── main.h
│   │   ├── pid.h                  # PID 结构体定义
│   │   └── ...
│   ├── application/
│   │   ├── CAN_receive.c/.h       # CAN 通信、M3508 反馈解析、DM4340 MIT 控制
│   │   ├── remote_control.c/.h    # SBUS 协议解析、DMA 双缓冲
│   │   ├── BMI088driver.c/.h      # BMI088 SPI 驱动（加速度计 + 陀螺仪）
│   │   └── bsp_can.c/.h           # CAN 底层过滤器配置
│   └── Startup/
│       └── startup_stm32f407ighx.s  # 启动文件
├── Drivers/
│   ├── CMSIS/                     # ARM CMSIS 核心库
│   └── STM32F4xx_HAL_Driver/      # ST HAL 驱动库
├── LED.ioc                        # STM32CubeMX 工程配置
├── CMakeLists.txt                 # ARM GCC 交叉编译配置
├── STM32F407IGHX_FLASH.ld         # Flash 链接脚本
├── STM32F407IGHX_RAM.ld           # RAM 链接脚本
└── config/
    └── stlink.cfg                 # ST-Link 调试配置
```

## 控制模式

| 模式 | 遥控器右拨杆 | 说明 |
|------|-------------|------|
| **type=3** | 中档 / 初始 | 失能模式，所有电机停止 |
| **type=1** | 上拨 | 遥控模式，摇杆控制底盘全向移动 |
| **type=0** | 下拨 | 初始化模式，IMU 零偏校准 |
| **type=2** | 校准完成后自动进入 | 自动追球 + 遥控融合模式 |

### 自动追球模式 (type=2)

融合上位机视觉坐标与遥控器输入：

```c
Vx = -(uart_x / 100.0f) + 遥控器Y轴    // 前后运动 = 视觉X补偿 + 手动
Vy = 遥控器X轴 + 陀螺仪航向PID补偿      // 旋转 = 手动 + 自动回正
Wz = -(uart_y / 100.0f) + 遥控器Z轴    // 左右运动 = 视觉Y补偿 + 手动
```

## 硬件配置

### MCU

- **型号**：STM32F407IGHx
- **主频**：168 MHz（HSE 12MHz + PLL）
- **Flash**：1 MB
- **RAM**：192 KB

### 外设映射

| 外设 | 功能 | 说明 |
|------|------|------|
| CAN1 | M3508 底盘电机 | 4 路速度闭环 |
| CAN2 | DM4340 执行电机 | 3 路 MIT 模式控制 |
| USART1 | 上位机通信 | 接收视觉坐标 / 调试输出 |
| USART3 | SBUS 遥控器 | DMA 双缓冲接收 |
| SPI1 | BMI088 IMU | 加速度计 + 陀螺仪 |
| TIM10 | 系统定时 | 控制节拍 |
| PB12 | GPIO 中断 | 外部触发信号（50ms 消抖） |
| PH10/PH11 | LED 指示 | 状态指示灯 |

### 电机参数

| 电机 | 型号 | 控制方式 | 用途 |
|------|------|---------|------|
| 底盘电机 x4 | M3508 | CAN1 速度环 PID | 麦克纳姆轮驱动 |
| 执行电机 x3 | DM4340 | CAN2 MIT 模式 | 击球/发球执行机构 |

## 环境要求

| 工具 | 说明 |
|------|------|
| ARM GCC | `arm-none-eabi-gcc` 交叉编译器 |
| CMake | >= 3.30 |
| STM32CubeMX | 外设配置（可选，用于修改 .ioc） |
| ST-Link | 调试下载器 |
| OpenOCD / STM32CubeProgrammer | 烧录工具 |

## 编译与烧录

### 编译

```bash
mkdir -p cmake-build-debug && cd cmake-build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
```

编译产物：
- `LED.elf` — ELF 可执行文件
- `LED.hex` — Intel HEX 烧录文件
- `LED.bin` — 二进制烧录文件

### 烧录

```bash
# 使用 OpenOCD + ST-Link
openocd -f config/stlink.cfg -c "program cmake-build-debug/LED.elf verify reset exit"

# 或使用 STM32CubeProgrammer
STM32_Programmer_CLI -c port=SWD -w cmake-build-debug/LED.bin 0x08000000 -v -rst
```

## 通信协议

### 上位机 → STM32（USART1）

```
格式:  "{x},{y}\n"
示例:  "120,-50\n"
含义:  排球相对画面中心的偏移量
波特率: 115200 8N1
```

### 串口调试输出

```
格式:  DM4340 电机状态
内容:  position, target_position, speed, current
```

## 麦克纳姆轮运动学

```
wheel1 = -(Vx + Vy + Wz) × speed_scale    // 右前
wheel2 =  (Vx - Vy - Wz) × speed_scale    // 左前
wheel3 = -(Vx + Vy - Wz) × speed_scale    // 右后
wheel4 =  (Vx - Vy + Wz) × speed_scale    // 左后
```

其中 `speed_scale = 1000 × (遥控器速度通道 + 1)`，实现实时速度倍率调节。

## 许可证

Apache License 2.0
