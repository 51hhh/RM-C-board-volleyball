# USB虚拟串口
https://blog.csdn.net/qq_49053936/article/details/142363059

# 上位机与嵌入式链路验证

本目录提供 RC 转发链路的真实连接验证脚本。

## 快速开始

```bash
# 安装依赖
pip3 install pyserial

# 1) 解析逻辑自检（不依赖硬件）
python3 scripts/rc_forward_sniff.py --self-test

# 2) 查看可用端口
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

# 3) 监听 24 字节固定帧（示例：/dev/ttyACM0）
python3 scripts/rc_forward_sniff.py /dev/ttyACM0 --verbose
```

## 脚本功能

- 帧长度固定 `24` 字节。
- 自动按 `0xA55A` 同步。
- 校验 `version/payload_len`。
- 校验 CRC16/Modbus。
- 统计：
  - total：总收包尝试
  - good：有效帧
  - bad(len/crc/magic)：解析失败计数
  - lost：按 seq 估算丢包
  - jitter：相邻帧接收间隔

## 真实链路测试（建议顺序）

### A. 烧录并确认固件
```bash
cd /home/rick/desktop/RM-C-board-volleyball
# 编译
mkdir -p cmake-build-debug
cd cmake-build-debug
cmake .. -DCMAKE_BUILD_TYPE=Debug
cmake --build . -j4

# 仅在当前会话不被占用 debug 工具时执行（否则会失败）
probe-rs download LED.elf --chip STM32F407IGHx --protocol swd --verify
# 或
# openocd -f config/daplink.cfg -c "program LED.elf verify reset exit"
```

### B. 检查 USB CDC 串口是否枚举
```bash
lsusb
ls -l /dev/ttyACM* /dev/ttyUSB*
dmesg | tail -n 80
```

### C. 启动监听
```bash
python3 scripts/rc_forward_sniff.py /dev/ttyACM0 --verbose
# 或使用 --raw 打印原始十六进制
```

## 典型故障排查

1. 监听不到设备：
   - 确认 USB data 线、开发板供电、CDC 端口是否启用。
   - 检查是否有旧的 openocd / probe-rs 独占 probe。
   - 检查 `lsusb` 是否仍显示 0d28:0204。

2. 全部帧 bad_crc：
   - 检查固件版本是否已更新（确认 `RC_FORWARD_FRAME_LEN == 24`）。
   - 检查上位机是否按小端 `struct` 解析。

3. 帧有空洞丢失：
   - 检查电脑 USB 架构负载；提升串口读取轮询频率。
   - 上位机若长时间阻塞写盘/打印，先降频（去掉 `--verbose`，只记录统计）。

## MCU 端联调建议（已适配当前固件）

- 固定帧定义：
  - 魔数：`0xA55A`
  - 长度：`24`
  - 版本：`1`
  - CRC16：Modbus-16，低位优先
- 字段说明见 `Core/application/rc_forward.h`

## 注意

如果你这台机器当前不能访问探针（无权限/占用），先在有权限设备上执行烧录，待固件更新后再回到这里做监听。
