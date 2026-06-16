# 调试脚本

本目录包含用于 STM32F407 嵌入式系统调试的自动化脚本。

## 脚本列表

### auto_test_can.py

**功能**：自动化 CAN 总线电机通信测试

**描述**：
- 自动连接 OpenOCD
- 加载固件到 STM32F407
- 运行程序 5 秒
- 采集所有电机的 CAN 反馈数据
- 生成测试报告，显示哪些电机在线

**使用方法**：
```bash
# 确保 OpenOCD 正在运行
openocd -f ../../config/daplink.cfg &

# 运行测试
python3 auto_test_can.py
```

**输出示例**：
```
=== M3508 Chassis Motors (CAN1) ===
Motor1 [0x201]: ecd=1870, speed=0 rpm, current=-22, temp=27
Motor2 [0x202]: ecd=2432, speed=0 rpm, current=18, temp=27
Motor3 [0x203]: ecd=3528, speed=0 rpm, current=23, temp=26
Motor4 [0x204]: ecd=0, speed=0 rpm, current=0, temp=0

=== DM4340 Motors (CAN2) ===
DM1 [ID 1]: pos=0.000, speed=0.000, current=0.000
DM2 [ID 2]: pos=0.000, speed=0.000, current=0.000
DM3 [ID 3]: pos=0.000, speed=0.000, current=0.000

>>> Analysis:
  M3508 motors responding: 3/4
  DM4340 motors responding: 0/3
```

**依赖**：
- Python 3
- gdb-multiarch
- OpenOCD
- 已编译的 `cmake-build-debug/LED.elf`

---

### debug_can.sh

**功能**：交互式 CAN 调试会话启动脚本

**描述**：
- 自动启动 OpenOCD（如果未运行）
- 启动 GDB 并加载预定义的调试命令
- 提供交互式命令行，方便查看电机状态

**使用方法**：
```bash
./debug_can.sh
```

**可用的 GDB 命令**：
- `print_all_motors` - 显示所有电机状态
- `print_m3508_motors` - 显示 M3508 底盘电机
- `print_dm4340_motors` - 显示 DM4340 执行电机
- `monitor_can` - 启动连续监控模式
- `c` - 继续执行
- `Ctrl+C` - 暂停执行

---

### auto_test_can.expect (备用)

**功能**：基于 Expect 的自动化测试脚本（需要安装 expect）

**描述**：
类似于 `auto_test_can.py`，但使用 Expect 脚本语言实现。如果系统已安装 expect，可以作为替代方案。

**使用方法**：
```bash
# 安装 expect（如果需要）
sudo apt install expect

# 运行脚本
./auto_test_can.expect
```

## 通用调试流程

### 1. 快速测试所有电机连接

```bash
# 一键测试
cd /home/rick/desktop/RM-C-board-volleyball
openocd -f config/daplink.cfg &
sleep 2
python3 scripts/debug/auto_test_can.py
```

### 2. 交互式调试特定问题

```bash
# 启动交互式调试
cd /home/rick/desktop/RM-C-board-volleyball
./scripts/debug/debug_can.sh

# 在 GDB 中
(gdb) monitor_can          # 开始监控
# 按 Ctrl+C 暂停
(gdb) print_all_motors     # 查看状态
(gdb) c                    # 继续
```

### 3. 手动 GDB 调试

```bash
# 终端 1：启动 OpenOCD
openocd -f config/daplink.cfg

# 终端 2：启动 GDB
gdb-multiarch cmake-build-debug/LED.elf
(gdb) target remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) b HAL_CAN_RxFifo0MsgPendingCallback
(gdb) continue
```

## 故障排查

### OpenOCD 无法连接

```bash
# 检查 USB 设备
lsusb | grep -E "0d28:0204|stlink"

# 测试 OpenOCD
openocd -f ../../config/daplink.cfg -c "init; targets; exit"
```

### Python 脚本执行失败

```bash
# 确保脚本有执行权限
chmod +x auto_test_can.py

# 检查 Python 版本
python3 --version  # 需要 Python 3.6+

# 检查依赖
which gdb-multiarch
which openocd
```

### 电机无反馈

1. 确认电机供电正常
2. 检查 CAN 总线连接（CAN_H、CAN_L、GND）
3. 确认 120Ω 终端电阻
4. 使用示波器检查 CAN 总线信号
5. 确认电机 ID 配置正确

## 更多信息

详细的调试指南请参阅：[../../docs/DEBUG.md](../../docs/DEBUG.md)
