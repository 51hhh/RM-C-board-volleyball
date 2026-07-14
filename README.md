# RM-C-board-volleyball 排球机器人底盘控制系统

[![Platform](https://img.shields.io/badge/Platform-STM32F407-red.svg)]()
[![Language](https://img.shields.io/badge/Language-C-blue.svg)]()

基于 **STM32F407IGHx**（168MHz, Cortex-M4F）的排球机器人嵌入式控制系统。

> **分支说明**：`serve-vehicle-A` 是发球车 A 的开发分支，集成了底盘全向运动 + 发球击球机构。

---

## 硬件拓扑

```
      SBUS 遥控                    BMI088 IMU
    (USART3 DMA)                  (SPI1)
         │                            │
         ▼                            ▼
┌─────────────────────────────────────────┐
│           STM32F407IGHx                 │
│                                         │
│  ┌──────────┐  ┌──────────┐            │
│  │ 底盘控制  │  │ 发球控制  │           │
│  │ 麦轮逆解  │  │ 状态机   │           │
│  │ 速度环PID │  │ 双环级联 │           │
│  └────┬─────┘  └────┬─────┘            │
│       │              │                  │
│    CAN1 (1Mbps)   CAN2 (1Mbps)         │
│       │              │                  │
│  4× M3508        3× M3508       PI7   PI6
│  底盘电机        击球臂A/B      电磁铁 光电开关
│  (0x201-204)     抛球电机
│                  (0x201-203)
└─────────────────────────────────────────┘
```

---

## 控制说明

### 遥控器布局

| 拨杆 | 功能 |
|------|------|
| **右拨杆** | 底盘模式切换 |
| **左拨杆** | 发球状态切换 |

### 底盘模式（右拨杆）

| 模式 | 拨杆位置 | 说明 |
|------|---------|------|
| Mode 3 失能 | 中档 / 初始 | 所有电机停止 |
| Mode 1 遥控 | 上拨 | 摇杆控制底盘全向移动 |
| Mode 3 失能 | 下拨 | IMU 未固定期间暂按失能处理，校准/自动模式不进入 |

### 发球状态（左拨杆）

| 状态 | 拨杆位置 | 动作 |
|------|---------|------|
| **IDLE** | 上拨 | 臂归零，抛球归零，电磁铁吸合 |
| **PREPARE** | 中档 | 臂慢速转到蓄力位，抛球转到蓄力位，电磁铁持续吸合 |
| **SERVE** | 下拨 | 电磁铁释放抛球 → 光电开关检测 → 臂开环击打 → 过位阻尼回零 |

### 发球时序

```
左拨杆下拨 → 电磁铁释放(PI7↓)
    → 球下落通过光电开关(PI6) 第 1 次（计数）
    → 延时 → 臂开环击打(±16000电流)
    → 臂到达击球断流位 → 低增益主动回零并施加速度阻尼
    → 若释放 3 秒仍无光电触发 → 锁存保护并自动回零
```

---

## 编译与烧录

### 编译

```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
```

产物：`build/LED.elf`、`build/LED.hex`、`build/LED.bin`

### 烧录（probe-rs）

```bash
probe-rs download --chip STM32F407IGHx build/LED.elf
```

### GDB 调试

```bash
# 终端1
probe-rs gdb --chip STM32F407IGHx

# 终端2
gdb-multiarch -x scripts/debug/check_can2_rc.gdb build/LED.elf
```

---

## 关键参数调参

所有电机 PID、目标位置、电流限幅集中在 `Core/application/motor_config.h`，分状态独立配置：

| 参数组 | 说明 |
|--------|------|
| `CHASSIS_*` | 底盘四轮速度环 |
| `ARM_STATE1_*` | 击球臂 IDLE 归位保持（软参数防抖动） |
| `ARM_STATE2_*` | 击球臂 PREPARE 蓄力 |
| `ARM_STATE3_HOLD_*` | 击球臂 SERVE 光电检测前保持 |
| `ARM_STATE3_STRIKE_*` | 击球臂开环击打电流和断流位 |
| `TOSS_STATE1_*` | 抛球电机 IDLE 归位保持 |
| `TOSS_STATE2_*` | 抛球电机 PREPARE 蓄力 |
| `TOSS_STATE3_HOLD_*` | 抛球电机 SERVE 保持 |

其他标定量在 `Core/application/striker.c` 顶部：
- 光电开关引脚、有效电平、消抖时间、击打延时
- 电磁铁引脚、有效电平

---

## 项目结构

```
Core/
├── Src/
│   ├── main.c              # 入口：HAL初始化 → 控制链启动 → while(1)
│   ├── scheduler.c/h        # TIM6@1kHz 时基 + 非阻塞延时事件
│   ├── pid.c/h              # 通用增量式 PID
│   └── ...
├── application/
│   ├── robot_control.c/h    # 控制编排：底盘状态机 + 遥控 + IMU + 遥测
│   ├── chassis.c/h          # 麦轮逆解 + 四轮速度环 + CAN1 下发
│   ├── striker.c/h          # 发球机构：三状态 + 双环级联 ± 光电触发 + CAN2 下发
│   ├── CAN_receive.c/h      # M3508/DM4340 CAN 收发
│   ├── remote_control.c/h   # SBUS 协议解析
│   ├── host_comm.c/h        # 上位机 UART 坐标接收
│   ├── imu_filter.c/h       # 三轴卡尔曼滤波
│   ├── led_indicator.c/h    # RGB LED 状态指示
│   └── motor_config.h       # ⭐ 全部电机参数集中配置
├── Inc/                     # HAL 外设头文件
└── Startup/                 # 启动汇编
```

---

## 遥控器 → SBUS 字节映射

| 字节 | 含义 | 典型值 |
|------|------|--------|
| `rc.s[0]` | 右拨杆 | 1=上, 2=下, 3=中, 0=初始 |
| `rc.s[1]` | 左拨杆 | 1=上, 2=下, 3=中, 0=初始 |
| `rc.ch[0]` | Wz 旋转 | -660 ~ +660 |
| `rc.ch[1]` | Vx 前后 | -660 ~ +660 |
| `rc.ch[2]` | Vy 左右 | -660 ~ +660 |
| `rc.ch[3]` | 速度倍率 | -660 ~ +660 |

---

## 远程仓库

| 名称 | 地址 |
|------|------|
| `origin` | git@github.com:51hhh/RM-C-board-volleyball.git |
| `me` | git@github.com:r0semaryrabb1t/RM-C-board-volleyball.git |

# 使用 OpenOCD + ST-Link
openocd -f config/stlink.cfg -c "program cmake-build-debug/LED.elf verify reset exit"

# 或使用 STM32CubeProgrammer
STM32_Programmer_CLI -c port=SWD -w cmake-build-debug/LED.bin 0x08000000 -v -rst
```

> **注意**：本项目当前使用 **DAPLink/CMSIS-DAP** 调试器，使用 `config/daplink.cfg` 配置文件。如果使用 ST-Link，请改用 `config/stlink.cfg`。

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

## 调试与测试

### 快速测试 CAN 电机连接

```bash
# 自动化测试所有电机
openocd -f config/daplink.cfg &
python3 scripts/debug/auto_test_can.py
```

详细的调试指南请参阅 [docs/DEBUG.md](docs/DEBUG.md)。

## 常见问题

| 问题 | 解决方案 |
|------|---------|
| 编译报错找不到交叉编译器 | `sudo apt install gcc-arm-none-eabi` |
| 烧录失败 | 检查调试器连接（DAPLink/ST-Link），确认供电正常 |
| OpenOCD 连接失败 | 检查 `lsusb` 输出，确认使用正确的配置文件（daplink.cfg 或 stlink.cfg） |
| 电机无反应 | 检查 CAN 接线、终端电阻，确认电机 ID 正确，使用调试脚本测试 |
| 遥控器无响应 | 检查 SBUS 接收器接线（USART3），确认遥控器已对频 |
| 上位机坐标不生效 | 确认拨杆处于 type=2 模式，检查串口波特率 115200 |
| DM4340 电机 MIT 模式配置 | 通过 DM 上位机工具设置电机 ID 和控制模式，确认已发送使能命令 |
| GDB 调试无符号 | 确认使用 Debug 构建：`cmake -DCMAKE_BUILD_TYPE=Debug ..` |

## 许可证

本项目基于 [Apache License 2.0](http://www.apache.org/licenses/LICENSE-2.0) 开源。
