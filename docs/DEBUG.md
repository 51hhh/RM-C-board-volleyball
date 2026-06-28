# 调试指南

## 硬件调试设置

### 调试器配置

本项目支持两种调试器：

1. **DAPLink/CMSIS-DAP**（当前使用）
   - USB VID:PID: 0x0d28:0x0204
   - 配置文件: `config/daplink.cfg`
   - OpenOCD 命令: `openocd -f config/daplink.cfg`

2. **ST-Link**（备用）
   - 配置文件: `config/stlink.cfg`
   - OpenOCD 命令: `openocd -f config/stlink.cfg`

### 连接确认

检查调试器连接：
```bash
# 检查 USB 设备
lsusb | grep -E "stlink|0d28:0204|mbed"

# 测试 OpenOCD 连接
openocd -f config/daplink.cfg -c "init; targets; exit"
```

检查目标板 CDC（当前固件描述符：VID:PID=0483:5760）：
```bash
lsusb | grep -i "0483:5760\|stm32"
lsusb -d 0483:5760 -v | head -n 40
```

## GDB 调试

### 基本调试流程

1. **启动 OpenOCD 服务器**（终端1）：
```bash
openocd -f config/daplink.cfg
```

2. **启动 GDB**（终端2）：
```bash
gdb-multiarch cmake-build-debug/LED.elf
```

3. **GDB 基本命令**：
```gdb
# 连接到 OpenOCD
target remote localhost:3333

# 复位并停止
monitor reset halt

# 加载程序
load

# 设置断点
b main
b HAL_CAN_RxFifo0MsgPendingCallback

# 运行程序
continue

# 单步执行
next       # 下一行（不进入函数）
step       # 下一行（进入函数）
finish     # 执行完当前函数

# 查看变量
print variable_name
print motor_chassis[0]
print DM4340_Date[0]

# 查看内存
x/10x 0x20000000

# 继续执行
continue

# 中断执行
Ctrl+C

# 退出
quit
```

### 常用调试场景

#### 1. 查看电机状态

```gdb
# 定义快捷命令
define print_m3508
    printf "=== M3508 Motors ===\n"
    printf "M1: ecd=%u speed=%d rpm\n", motor_chassis[0].ecd, motor_chassis[0].speed_rpm
    printf "M2: ecd=%u speed=%d rpm\n", motor_chassis[1].ecd, motor_chassis[1].speed_rpm
    printf "M3: ecd=%u speed=%d rpm\n", motor_chassis[2].ecd, motor_chassis[2].speed_rpm
    printf "M4: ecd=%u speed=%d rpm\n", motor_chassis[3].ecd, motor_chassis[3].speed_rpm
end

# 使用命令
print_m3508
```

#### 2. 监控 CAN 通信

```gdb
# 在 CAN 接收中断设置断点
b HAL_CAN_RxFifo0MsgPendingCallback

# 继续执行
continue

# 每次触发断点时查看数据
print hcan->Instance
info registers
```

#### 3. 调试 IMU 数据

```gdb
# 查看 BMI088 数据
print gyro[0]
print gyro[1]
print gyro[2]
print accel[0]
print accel[1]
print accel[2]
```

## 自动化测试脚本

### CAN 电机通信测试

位置: `scripts/debug/auto_test_can.py`

**功能**：
- 自动加载固件
- 运行程序 5 秒
- 检查所有电机的 CAN 通信状态
- 生成测试报告

**使用方法**：
```bash
cd /home/rick/desktop/RM-C-board-volleyball

# 确保 OpenOCD 正在运行
openocd -f config/daplink.cfg &

# 运行测试
python3 scripts/debug/auto_test_can.py
```

**输出示例**：
```
=== M3508 Chassis Motors (CAN1) ===
Motor1 [0x201]: ecd=1870, speed=0 rpm, current=-22, temp=27
Motor2 [0x202]: ecd=2432, speed=0 rpm, current=18, temp=27
Motor3 [0x203]: ecd=3528, speed=0 rpm, current=23, temp=26
Motor4 [0x204]: ecd=0, speed=0 rpm, current=0, temp=0

>>> Analysis:
  M3508 motors responding: 3/4
  DM4340 motors responding: 0/3
```

### 交互式调试脚本

位置: `scripts/debug/debug_can.sh`

**使用方法**：
```bash
./scripts/debug/debug_can.sh
```

启动后提供交互式 GDB 会话，包含预定义的电机查看命令。

## 调试技巧

### 1. 查找内存泄漏

使用 GDB 监控堆栈：
```gdb
info stack
backtrace
```

### 2. 实时变量监控

```gdb
# 设置显示刷新
set print pretty on

# 监控变量
display motor_chassis[0].speed_rpm
display gyro[2]

# 继续执行时自动显示
continue
```

### 3. 条件断点

```gdb
# 仅当速度大于 100 时触发
b CAN_cmd_chassis if motor_chassis[0].speed_rpm > 100
```

### 4. 调试优化代码

编译时添加调试符号并降低优化等级：
```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```

## 常见问题排查

### OpenOCD 连接失败

**问题**：`Error: open failed`

**解决方案**：
1. 检查 USB 连接
2. 确认调试器供电
3. 检查 udev 规则：
   ```bash
   ls -l /etc/udev/rules.d/*stlink*
   ```
4. 重新插拔调试器
5. 检查权限：
   ```bash
   sudo usermod -a -G dialout $USER
   # 注销重新登录
   ```

### GDB 无法加载符号

**问题**：`No symbol table is loaded`

**解决方案**：
```bash
# 重新编译带调试符号
cd cmake-build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make clean && make

# 确认 ELF 文件包含符号
file LED.elf | grep "not stripped"
```

### CAN 总线无通信

**排查步骤**：
1. 检查 CAN 收发器供电（通常 5V）
2. 确认 CAN_H 和 CAN_L 连接正确
3. 检查 120Ω 终端电阻
4. 用 GDB 检查 CAN 初始化：
   ```gdb
   b MX_CAN1_Init
   b MX_CAN2_Init
   continue
   print hcan1
   print hcan2
   ```

### 电机无响应

**M3508 电机**：
1. 检查 CAN1 连接
2. 确认电机 ID (0x201-0x204)
3. 检查电机供电（24V）
4. 在 GDB 中查看：
   ```gdb
   print motor_chassis[0]
   print motor_chassis[1]
   print motor_chassis[2]
   print motor_chassis[3]
   ```

**DM4340 电机**：
1. 检查 CAN2 连接
2. 确认电机已使能（需要发送使能命令）
3. 检查电机工作模式（MIT 模式）
4. 在 GDB 中查看：
   ```gdb
   print DM4340_Date[0]
   print DM4340_Date[1]
   print DM4340_Date[2]
   ```

## 性能分析

### 使用 SystemView

1. 在代码中添加 SEGGER SystemView 探针
2. 配置 RTT
3. 使用 SystemView 分析任务切换和中断

### 使用 GDB 测量执行时间

```gdb
# 设置断点
b function_start
commands
  silent
  set $start = HAL_GetTick()
  continue
end

b function_end
commands
  silent
  set $end = HAL_GetTick()
  printf "Execution time: %d ms\n", $end - $start
  continue
end
```

## 远程调试

### 通过网络调试

OpenOCD 支持网络连接：
```bash
# 服务器端
openocd -f config/daplink.cfg

# 客户端
gdb-multiarch cmake-build-debug/LED.elf
(gdb) target remote 192.168.1.100:3333
```

### 使用 VSCode 调试

在 `.vscode/launch.json` 中配置：
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug STM32",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/cmake-build-debug/LED.elf",
            "miDebuggerPath": "gdb-multiarch",
            "miDebuggerServerAddress": "localhost:3333",
            "setupCommands": [
                {
                    "text": "monitor reset halt"
                },
                {
                    "text": "load"
                }
            ]
        }
    ]
}
```

## 日志和跟踪

### UART 调试输出

使用 DMA 队列发送调试信息（已实现）：
```c
char buffer[256];
snprintf(buffer, sizeof(buffer), "Speed: %d\n", speed);
// DMA 队列自动发送
```

### SWO 跟踪

配置 SWO 输出：
```bash
openocd -f config/daplink.cfg -c "tpiu config internal /tmp/swo.log uart off 168000000"
```

查看输出：
```bash
tail -f /tmp/swo.log
```
