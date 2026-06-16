# 开发指南

本文档包含项目开发、编译、调试和烧录的完整流程。

## 📋 目录

- [开发环境](#开发环境)
- [编译流程](#编译流程)
- [烧录方法](#烧录方法)
- [调试方法](#调试方法)
- [CubeMX 使用注意事项](#cubemx-使用注意事项)

---

## 开发环境

### 必需工具

| 工具 | 版本要求 | 说明 |
|------|---------|------|
| **ARM GCC** | 任意版本 | ARM 交叉编译工具链 |
| **CMake** | >= 3.10 | 构建系统 |
| **OpenOCD** | >= 0.10 | 调试和烧录工具 |
| **ST-Link** | 硬件 | 调试器/烧录器 |
| **STM32CubeMX** | >= 6.11 (可选) | 外设配置工具 |

### 安装 ARM GCC 工具链

#### Ubuntu/Debian
```bash
sudo apt update
sudo apt install gcc-arm-none-eabi gdb-multiarch
```

#### Arch Linux
```bash
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-gdb
```

#### macOS
```bash
brew install --cask gcc-arm-embedded
```

#### Windows
下载并安装 [GNU Arm Embedded Toolchain](https://developer.arm.com/downloads/-/gnu-rm)

### 验证安装

```bash
# 检查 ARM GCC 版本
arm-none-eabi-gcc --version

# 检查 OpenOCD 版本
openocd --version

# 检查 CMake 版本
cmake --version
```

---

## 编译流程

### 1. 清理构建目录（可选）

```bash
rm -rf cmake-build-debug
```

### 2. 配置 CMake

```bash
mkdir -p cmake-build-debug
cd cmake-build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

**可用的构建类型**：
- `Debug` - 调试版本（包含调试符号，优化级别 -Og）
- `Release` - 发布版本（优化级别 -O3，体积更小）

### 3. 编译项目

```bash
# 使用所有 CPU 核心编译
make -j$(nproc)

# 或指定核心数
make -j4
```

### 4. 查看编译产物

编译成功后会生成：
```
cmake-build-debug/
├── LED.elf        # ELF 可执行文件（用于 GDB 调试）
├── LED.hex        # Intel HEX 格式（常用烧录格式）
├── LED.bin        # 二进制格式（原始固件）
└── LED.map        # 内存映射文件
```

### 5. 查看固件大小

```bash
arm-none-eabi-size cmake-build-debug/LED.elf
```

输出示例：
```
   text    data     bss     dec     hex filename
  72708    1772    8440   82920   143e8 LED.elf
```

- `text` - 代码段 (Flash)
- `data` - 已初始化数据 (Flash → RAM)
- `bss` - 未初始化数据 (RAM)

---

## 烧录方法

### 方法 1: 使用 OpenOCD（推荐）

#### 基本烧录命令

```bash
openocd -f config/stlink.cfg \
        -c "program cmake-build-debug/LED.elf verify reset exit"
```

**参数说明**：
- `-f config/stlink.cfg` - 使用 ST-Link 调试器配置
- `program` - 烧录固件
- `verify` - 烧录后验证
- `reset` - 烧录完成后复位 MCU
- `exit` - 操作完成后退出

#### 烧录 HEX 文件

```bash
openocd -f config/stlink.cfg \
        -c "program cmake-build-debug/LED.hex verify reset exit"
```

#### 烧录 BIN 文件（需指定地址）

```bash
openocd -f config/stlink.cfg \
        -c "program cmake-build-debug/LED.bin 0x08000000 verify reset exit"
```

#### 常见问题

**问题 1**: `Error: libusb_open() failed with LIBUSB_ERROR_ACCESS`
```bash
# 解决方法：添加 udev 规则
sudo cp /usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**问题 2**: `Error: open failed`
```bash
# 检查 ST-Link 连接
lsusb | grep ST-Link

# 尝试使用 sudo
sudo openocd -f config/stlink.cfg ...
```

---

### 方法 2: 使用 STM32CubeProgrammer

#### 命令行方式

```bash
STM32_Programmer_CLI -c port=SWD \
                     -w cmake-build-debug/LED.bin 0x08000000 \
                     -v -rst
```

**参数说明**：
- `-c port=SWD` - 使用 SWD 接口连接
- `-w <file> <address>` - 写入固件到指定地址
- `-v` - 验证烧录内容
- `-rst` - 复位 MCU

#### GUI 方式

1. 打开 STM32CubeProgrammer
2. 选择 **ST-LINK** 连接方式
3. 点击 **Connect**
4. 选择 `LED.hex` 或 `LED.bin` 文件
5. 点击 **Start Programming**

---

## 调试方法

### 使用 OpenOCD + GDB

#### 1. 启动 OpenOCD 服务器

```bash
# 在终端 1 运行
openocd -f config/stlink.cfg
```

输出示例：
```
Open On-Chip Debugger 0.11.0
Info : Listening on port 3333 for gdb connections
```

#### 2. 连接 GDB

```bash
# 在终端 2 运行
arm-none-eabi-gdb cmake-build-debug/LED.elf

# 或使用 gdb-multiarch (某些发行版)
gdb-multiarch cmake-build-debug/LED.elf
```

#### 3. GDB 命令

```gdb
# 连接到 OpenOCD
(gdb) target extended-remote :3333

# 加载固件
(gdb) load

# 复位并停止
(gdb) monitor reset halt

# 设置断点
(gdb) break main
(gdb) break Core/Src/main.c:241

# 继续执行
(gdb) continue

# 单步执行
(gdb) step      # 单步进入
(gdb) next      # 单步跳过
(gdb) finish    # 执行到函数返回

# 查看变量
(gdb) print uart_x
(gdb) print /x uart_y        # 十六进制
(gdb) display uart_x         # 持续显示

# 查看内存
(gdb) x/10x 0x20000000       # 查看 10 个字（十六进制）

# 查看调用栈
(gdb) backtrace
(gdb) bt

# 查看寄存器
(gdb) info registers
(gdb) info all-registers

# 复位 MCU
(gdb) monitor reset halt

# 退出
(gdb) quit
```

#### 4. GDB 初始化脚本（可选）

创建 `.gdbinit` 文件：
```bash
cat > .gdbinit << 'EOF'
# 连接到 OpenOCD
target extended-remote :3333

# 加载符号
file cmake-build-debug/LED.elf

# 禁用分页
set pagination off

# 启用 TUI 模式（可选）
# tui enable

# 自定义命令
define reload
  monitor reset halt
  load
  monitor reset halt
end

document reload
重新加载固件并复位
EOF
```

使用：
```bash
arm-none-eabi-gdb
(gdb) reload    # 使用自定义命令
```

---

### 串口调试

项目使用 **UART1** (115200 8N1) 作为调试输出。

#### Linux/macOS

```bash
# 查找设备
ls /dev/ttyUSB* /dev/ttyACM*

# 使用 screen
screen /dev/ttyUSB0 115200

# 使用 minicom
minicom -D /dev/ttyUSB0 -b 115200

# 使用 picocom
picocom -b 115200 /dev/ttyUSB0

# 退出 screen: Ctrl-A + K + Y
```

#### Windows

使用 **PuTTY** 或 **Tera Term**：
1. 选择串口（如 COM3）
2. 波特率：115200
3. 数据位：8
4. 停止位：1
5. 校验：无

---

## CubeMX 使用注意事项

### ⚠️ 重要：手动维护代码

CubeMX 重新生成时，**不在 USER CODE 保护区**的代码会被删除。

#### 需要手动添加的代码

**位置**: `Core/Src/main.c` → `SystemClock_Config()` 函数

**原因**: GDB 调试时避免时钟跑飞

**代码**:
```c
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // ⚠️ 每次 CubeMX 重新生成后需要手动添加以下代码 ⚠️
  // ========================================================
  // 先将时钟源选择为内部时钟（GDB调试时避免时钟跑飞）
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
      Error_Handler();
  }
  // ========================================================

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  // ... 后续代码保持不变
}
```

### CubeMX 工作流程

#### 1. 修改前备份

```bash
git add -A
git commit -m "备份：CubeMX 修改前"
```

#### 2. 使用 CubeMX 修改配置

1. 打开 `LED.ioc`
2. 修改外设配置
3. **Project Manager → Code Generator** 确认：
   - ✅ **Keep User Code when re-generating** (已勾选)
4. 点击 **GENERATE CODE**

#### 3. 修改后检查

```bash
# 查看差异
git diff

# 重点检查 main.c
git diff Core/Src/main.c
```

#### 4. 手动添加时钟配置代码

参考上方的代码片段。

#### 5. 编译验证

```bash
cd cmake-build-debug
make -j$(nproc)
```

#### 6. 提交修改

```bash
git add -A
git commit -m "feat: CubeMX 配置更新 + 手动恢复时钟配置"
```

---

## 常用命令速查

### 编译

```bash
# 快速编译
cd cmake-build-debug && make -j$(nproc)

# 清理重编
rm -rf cmake-build-debug && mkdir cmake-build-debug && \
cd cmake-build-debug && cmake .. && make -j$(nproc)

# 查看编译警告
make 2>&1 | grep warning
```

### 烧录

```bash
# OpenOCD 烧录
openocd -f config/stlink.cfg \
        -c "program cmake-build-debug/LED.elf verify reset exit"

# 快速烧录（假设 OpenOCD 已运行）
echo "program cmake-build-debug/LED.elf verify reset" | \
nc localhost 4444
```

### 调试

```bash
# 启动 OpenOCD
openocd -f config/stlink.cfg

# 启动 GDB（另一个终端）
arm-none-eabi-gdb cmake-build-debug/LED.elf \
  -ex "target extended-remote :3333" \
  -ex "load" \
  -ex "monitor reset halt"
```

### 串口监控

```bash
# screen
screen /dev/ttyUSB0 115200

# picocom
picocom -b 115200 /dev/ttyUSB0

# 同时记录日志
picocom -b 115200 /dev/ttyUSB0 | tee debug.log
```

---

## 故障排查

### 编译错误

| 错误 | 原因 | 解决方案 |
|------|------|---------|
| `arm-none-eabi-gcc: command not found` | 未安装工具链 | 安装 `gcc-arm-none-eabi` |
| `CMake Error: could not find CMAKE_MAKE_PROGRAM` | 未安装 make | 安装 `make` 或 `build-essential` |
| `undefined reference to...` | 链接错误 | 检查 CMakeLists.txt 源文件列表 |

### 烧录错误

| 错误 | 原因 | 解决方案 |
|------|------|---------|
| `Error: libusb_open() failed` | 权限问题 | 添加 udev 规则或使用 sudo |
| `Error: open failed` | ST-Link 未连接 | 检查硬件连接 |
| `Error: target not halted` | MCU 未停止 | 重新连接或复位 |

### 调试错误

| 错误 | 原因 | 解决方案 |
|------|------|---------|
| `Remote 'g' packet reply is too long` | GDB 架构不匹配 | 使用 `gdb-multiarch` |
| `Cannot access memory at address 0x...` | 地址非法 | 检查指针或内存映射 |
| `Remote connection closed` | OpenOCD 断开 | 重启 OpenOCD |

---

## 附录：config/stlink.cfg

如果项目中没有此文件，创建它：

```bash
mkdir -p config
cat > config/stlink.cfg << 'EOF'
# ST-Link 调试器配置
source [find interface/stlink.cfg]

# 传输方式：SWD
transport select hla_swd

# 目标芯片：STM32F4x
source [find target/stm32f4x.cfg]

# 复位配置
reset_config srst_only
EOF
```

---

## 参考资料

- [OpenOCD 官方文档](https://openocd.org/documentation/)
- [ARM GCC 工具链](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)
- [STM32CubeMX 用户手册](https://www.st.com/resource/en/user_manual/um1718-stm32cubemx-for-stm32-configuration-and-initialization-c-code-generation-stmicroelectronics.pdf)
- [GDB 调试指南](https://sourceware.org/gdb/current/onlinedocs/gdb/)

---

## 更新日志

- 2026-06-16: 创建开发指南文档
- 添加 ARM GCC 编译流程
- 添加 OpenOCD 烧录和调试方法
- 添加 CubeMX 使用注意事项
