# Claude Code 项目指南

## 项目概述

这是一个基于 STM32F407IGHx 的排球机器人底盘控制系统，使用 ARM GCC 工具链和 CMake 构建。

## 快速开始

### 编译固件
```bash
cd cmake-build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
```

### 烧录固件（使用 DAPLink）
```bash
openocd -f config/daplink.cfg -c "program cmake-build-debug/LED.elf verify reset exit"
```

### 调试
```bash
# 启动 OpenOCD 服务器
openocd -f config/daplink.cfg

# 在另一个终端连接 GDB
arm-none-eabi-gdb cmake-build-debug/LED.elf
(gdb) target remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) continue
```

## 硬件配置

- **MCU**: STM32F407IGHx (168MHz, 1MB Flash, 192KB RAM)
- **调试器**: DAPLink/CMSIS-DAP (USB VID:PID = 0x0d28:0x0204)
- **底盘电机**: 4x M3508 (CAN1)
- **执行电机**: 3x DM4340 (CAN2, MIT 模式)
- **IMU**: BMI088 (SPI1)
- **遥控器**: SBUS (USART3)
- **上位机通信**: UART (USART1, 115200bps)

## 代码结构

### 用户代码位置
所有用户代码必须放在 CubeMX 保护区内（`/* USER CODE BEGIN */` 和 `/* USER CODE END */` 之间），否则重新生成代码时会被覆盖。

### 核心模块
- `Core/Src/main.c` - 主控制循环、状态机、麦轮运动学、卡尔曼滤波
- `Core/Src/pid.c` - PID 控制器
- `Core/application/CAN_receive.c` - M3508 和 DM4340 电机控制
- `Core/application/remote_control.c` - SBUS 遥控器协议解析
- `Core/application/BMI088driver.c` - IMU 驱动

## 控制模式

| 模式 | 遥控器右拨杆 | 说明 |
|------|-------------|------|
| type=3 | 中档 | 失能模式 |
| type=1 | 上拨 | 遥控模式 |
| type=0 | 下拨 | IMU 校准（2秒后自动进入 type=2） |
| type=2 | 自动 | 自动追球 + 遥控融合模式 |

## 通信协议

### 上位机 → STM32 (USART1)
```
格式: "{out_x},{out_y}\n"
示例: "120,-50\n"
```

注意：`sscanf` 解析顺序为 `sscanf(buf, "%d,%d", &uart_y, &uart_x)`
- 第一个值 → uart_y → Wz (左右运动)
- 第二个值 → uart_x → Vx (前后运动)

## 开发注意事项

1. **修改硬件配置**: 使用 STM32CubeMX 打开 `LED.ioc`，修改后重新生成代码
2. **用户代码保护**: 始终在 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 之间编写代码
3. **PID 参数调试**: 在 `main.c` 中修改各电机的 PID 参数
4. **CAN ID 配置**: M3508 ID 范围 0x201-0x204，DM4340 ID 为 1-3
5. **IMU 校准**: 每次上电后切换到 type=0 模式进行零偏校准

## 常见任务

### 添加新的外设
1. 在 STM32CubeMX 中配置外设
2. 重新生成代码
3. 在 `Core/application/` 下创建驱动文件
4. 在 `CMakeLists.txt` 中添加源文件

### 调试 CAN 通信
- 查看 `CAN_receive.c` 中的电机反馈数据
- 使用串口输出调试信息（DMA 队列发送）

### 修改运动控制
- 麦克纳姆轮运动学在 `main.c` 的主循环中
- 速度倍率由遥控器速度通道控制

## 工具链版本

- ARM GCC: 14.2.1
- OpenOCD: 0.12.0
- CMake: 3.30+
- GDB: gdb-multiarch

## 调试工具

### 当前硬件配置
- **调试器**: DAPLink/CMSIS-DAP (USB VID:PID = 0x0d28:0x0204)
- **配置文件**: `config/daplink.cfg`
- **备用**: ST-Link (`config/stlink.cfg`)

### 快速调试命令

**编译并烧录**：
```bash
cd cmake-build-debug
make -j$(nproc)
openocd -f ../config/daplink.cfg -c "program LED.elf verify reset exit"
```

**测试 CAN 电机连接**：
```bash
# 自动化测试
openocd -f config/daplink.cfg &
python3 scripts/debug/auto_test_can.py
```

**交互式 GDB 调试**：
```bash
# 终端 1
openocd -f config/daplink.cfg

# 终端 2
gdb-multiarch cmake-build-debug/LED.elf
(gdb) target remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) b main
(gdb) continue
```

### 调试脚本

所有调试脚本位于 `scripts/debug/` 目录：

- `auto_test_can.py` - 自动化 CAN 电机通信测试（推荐）
- `debug_can.sh` - 交互式 GDB 调试会话
- `README.md` - 调试脚本详细说明

### 查看电机状态（GDB）

```gdb
# M3508 底盘电机 (CAN1)
print motor_chassis[0]  # Motor 1 (0x201)
print motor_chassis[1]  # Motor 2 (0x202)
print motor_chassis[2]  # Motor 3 (0x203)
print motor_chassis[3]  # Motor 4 (0x204)

# DM4340 执行电机 (CAN2)
print DM4340_Date[0]  # DM Motor 1
print DM4340_Date[1]  # DM Motor 2
print DM4340_Date[2]  # DM Motor 3

# IMU 数据
print gyro[0]   # X轴角速度
print gyro[1]   # Y轴角速度
print gyro[2]   # Z轴角速度
print accel[0]  # X轴加速度
print accel[1]  # Y轴加速度
print accel[2]  # Z轴加速度
```

### 常见调试任务

**检查 CAN 通信**：
在 `HAL_CAN_RxFifo0MsgPendingCallback` 设置断点，观察 CAN 消息接收。

**调试电机控制**：
在 `CAN_cmd_chassis` 和 `MD4340_motor_PID_Control` 设置断点，检查发送的控制命令。

**监控遥控器输入**：
在主循环中查看 `rc_ctrl` 结构体的值。

**详细调试指南**：请参阅 [docs/DEBUG.md](docs/DEBUG.md)
