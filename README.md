# RM-C-board-volleyball 排球机器人底盘控制系统

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](http://www.apache.org/licenses/LICENSE-2.0)
[![Language](https://img.shields.io/badge/Language-C-blue.svg)]()
[![Platform](https://img.shields.io/badge/Platform-STM32F407-red.svg)]()

基于 **STM32F407IGHx** 的排球机器人嵌入式控制系统，实现麦克纳姆轮全向底盘运动控制、DM4340 电机 MIT 模式控制、BMI088 IMU 姿态感知、SBUS 遥控器控制，并通过串口接收上位机视觉检测坐标实现自动追球。

> **项目定位**：本项目是排球机器人系统的**下位机控制模块**，需配合上位机视觉系统才能实现自动追球功能。上位机负责检测排球并发送坐标，本项目负责解析坐标并驱动底盘运动。

## 排球机器人系统全景

| 仓库 | 角色 | 说明 |
|------|------|------|
| [yolov8_volleyball](https://github.com/51hhh/yolov8_volleyball) | 上位机（推荐） | YOLOv8+NanoDet 双引擎，Boost.Asio 串口通信 |
| [nanodet_volleyball](https://github.com/51hhh/nanodet_volleyball) | 上位机 | NanoDet 专用版，终端坐标输出 |
| [Nanodet_OpenVINO](https://github.com/51hhh/Nanodet_OpenVINO) | 上位机（调试用） | NanoDet 可视化调试版本 |
| **RM-C-board-volleyball**（本项目） | 下位机 | STM32F407 麦克纳姆轮底盘控制 |

## 特性

- **麦克纳姆轮全向底盘**：4 轮独立 PID 速度闭环，支持全向移动与旋转
- **DM4340 电机控制**：CAN 总线 MIT 控制模式（可同时控制位置/速度/力矩），支持位置闭环
- **M3508 底盘电机**：CAN 通信，速度环 PID 控制
- **BMI088 六轴 IMU**：SPI 接口驱动，加速度 + 陀螺仪数据采集
- **卡尔曼滤波**：加速度计零偏估计与噪声滤波
- **陀螺仪航向保持**：Z 轴角速度积分 + PID 回正控制
- **SBUS 遥控器**：DMA 双缓冲接收，支持多档位状态切换
- **串口坐标接收**：USART1 接收上位机视觉系统发送的排球坐标，融合至底盘运动控制
- **UART DMA 发送队列**：环形缓冲区 + DMA 非阻塞发送，调试信息不阻塞主循环
- **3 种控制模式**：遥控模式 / 自动追球模式 / 失能模式
- **GPIO 中断触发**：PB12 下降沿触发 DM4340 电机动作序列（50ms 消抖）

## 系统架构

```
┌──────────────┐  UART 115200bps  ┌────────────────────────────────────┐
│  上位机       │ ───────────────▶ │  STM32F407IGHx (168MHz)           │
│  视觉检测系统 │  坐标 "v,u\n"    │                                    │
│              │                  │  ┌──────────┐   ┌──────────────┐  │
│ yolov8_      │                  │  │ SBUS遥控  │   │ BMI088 IMU   │  │
│ volleyball / │                  │  │ (USART3)  │   │ (SPI1)       │  │
│ nanodet_     │                  │  └─────┬────┘   └──────┬───────┘  │
│ volleyball   │                  │        │                │          │
└──────────────┘                  │        ▼                ▼          │
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
│   │   ├── bsp_can.c              # CAN 底层过滤器配置与初始化
│   │   └── ...                    # HAL 外设初始化（gpio, dma, can, spi, usart, tim）
│   ├── Inc/
│   │   ├── main.h / pid.h / bsp_can.h
│   │   └── ...
│   ├── application/
│   │   ├── CAN_receive.c/.h       # CAN 通信、M3508 反馈解析、DM4340 MIT 控制与位置闭环
│   │   ├── remote_control.c/.h    # SBUS 协议解析、DMA 双缓冲
│   │   ├── BMI088driver.c/.h      # BMI088 SPI 驱动（加速度计 + 陀螺仪）
│   │   ├── BMI088Middleware.c/.h  # BMI088 SPI 底层中间件
│   │   ├── BMI088reg.h            # BMI088 寄存器定义
│   │   ├── bsp_rc.c/.h            # 遥控器底层驱动
│   │   └── struct_typedef.h       # 类型定义
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

> 注：CubeMX 工程名为 `LED.ioc`，因此编译产物命名为 `LED.elf/hex/bin`。

## 控制模式

通过遥控器右拨杆切换控制模式：

| 模式 | 遥控器右拨杆 | SBUS 值 | 说明 |
|------|-------------|---------|------|
| **type=3** | 中档 / 初始 | `0x03` / `0x00` | 失能模式，所有电机停止 |
| **type=1** | 上拨 | `0x01` | 遥控模式，摇杆控制底盘全向移动 |
| **type=0** | 下拨 | `0x02` | 初始化模式：IMU 零偏校准（约 2 秒后自动进入 type=2） |
| **type=2** | 校准完成后自动进入 | — | 自动追球 + 遥控融合模式 |

### 自动追球模式 (type=2)

融合上位机视觉坐标与遥控器输入：

```c
// 上位机发送 "out_x,out_y\n"，sscanf 解析为: 第1个值→uart_y, 第2个值→uart_x
// 因此 uart_y 实际对应上位机的水平偏移，uart_x 对应垂直偏移

float Vx = -(uart_x / 100.0f) + y_normalized * 5;  // 前后运动
float Vy = x_normalized * 5 + wz_compensation;      // 旋转 = 手动 + 陀螺仪PID回正
float Wz = -(uart_y / 100.0f) + z_normalized * 5;   // 左右运动
```

> 注：遥控器摇杆归一化值（-1~1）乘以系数 5 放大控制量。`speed_scale = 1000 × (遥控器速度通道 + 1)` 实现速度倍率调节。

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
| CAN2 | DM4340 执行电机 | 3 路 MIT 模式控制 + 位置闭环 |
| USART1 | 上位机通信 | 接收视觉坐标 / DMA 调试输出 |
| USART3 | SBUS 遥控器 | DMA 双缓冲接收 |
| SPI1 | BMI088 IMU | 加速度计 + 陀螺仪 |
| TIM10 | 系统定时 | 控制节拍 |
| PB12 | GPIO 中断 | 下降沿触发 DM4340 动作序列（50ms 消抖） |
| PH10/PH11 | LED 指示 | 状态指示灯 |

### 电机参数

| 电机 | 型号 | 控制方式 | 用途 |
|------|------|---------|------|
| 底盘电机 x4 | M3508 | CAN1 速度环 PID | 麦克纳姆轮驱动 |
| 执行电机 x3 | DM4340 | CAN2 MIT 模式 + 位置闭环 PID | 击球/发球执行机构 |

> MIT 模式：源自 MIT Cheetah 项目的电机控制协议，允许单条 CAN 报文同时下发位置、速度、力矩三个控制量。

## 环境要求

### 软件工具

| 工具 | 说明 | 安装方式 |
|------|------|---------|
| ARM GCC | `arm-none-eabi-gcc` 交叉编译器 | `sudo apt install gcc-arm-none-eabi` |
| CMake | >= 3.30 | `sudo apt install cmake` |
| STM32CubeMX | 外设配置（可选，修改 .ioc 时需要） | [ST 官网下载](https://www.st.com/en/development-tools/stm32cubemx.html) |
| ST-Link | 调试下载器（硬件） | — |
| OpenOCD / STM32CubeProgrammer | 烧录工具 | `sudo apt install openocd` |

## 编译与烧录

### 编译

```bash
git clone https://github.com/51hhh/RM-C-board-volleyball.git
cd RM-C-board-volleyball
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

### 上位机 → STM32（USART1, 115200 8N1）

```
格式:  "{out_x},{out_y}\n"
示例:  "120,-50\n"
含义:  排球中心相对画面中心的偏移量（由上位机计算）
```

> **重要**：STM32 端 `sscanf` 解析顺序为 `sscanf(buf, "%d,%d", &uart_y, &uart_x)`，即接收到的**第一个值存入 `uart_y`**（映射到 Wz 左右运动），**第二个值存入 `uart_x`**（映射到 Vx 前后运动）。这是有意的坐标轴映射。

### 串口调试输出（DMA 队列发送）

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

> 麦克纳姆轮（Mecanum Wheel）是一种带有斜向滚子的特殊车轮，4 个麦轮组合可实现全向移动（前后、左右平移、原地旋转）。

## 常见问题

| 问题 | 解决方案 |
|------|---------|
| 编译报错找不到交叉编译器 | `sudo apt install gcc-arm-none-eabi` |
| 烧录失败 | 检查 ST-Link 连接，确认供电正常 |
| 电机无反应 | 检查 CAN 接线、终端电阻，确认电机 ID 正确 |
| 遥控器无响应 | 检查 SBUS 接收器接线（USART3），确认遥控器已对频 |
| 上位机坐标不生效 | 确认拨杆处于 type=2 模式，检查串口波特率 115200 |
| DM4340 电机 MIT 模式配置 | 通过 DM 上位机工具设置电机 ID 和控制模式 |

## 许可证

本项目基于 [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0) 开源。
