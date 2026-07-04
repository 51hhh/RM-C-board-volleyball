# 烧录说明

本文档记录本工程的标准编译和烧录流程。当前工程使用 STM32F407IGHx，推荐使用 probe-rs 通过 SWD 烧录。

## 快速烧录

在工程主目录执行：

```bash
cd /home/rosemaryrabbit/RM-C-board-volleyball
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
probe-rs download --chip STM32F407IGHx --protocol swd --verify build/LED.elf
probe-rs reset --chip STM32F407IGHx --protocol swd
```

其中 `probe-rs download` 负责下载程序，`probe-rs reset` 负责复位并启动目标板。

## 编译输出

工程名为 `LED`，标准构建目录为 `build/`。编译完成后会生成：

| 文件 | 用途 |
|------|------|
| `build/LED.elf` | 推荐烧录文件，包含符号信息，适合 probe-rs 下载和调试 |
| `build/LED.hex` | Intel HEX 文件，适合部分通用烧录工具 |
| `build/LED.bin` | 纯二进制文件，烧录时必须指定 Flash 起始地址 |
| `build/LED.map` | 链接映射文件，用于排查内存占用 |

## probe-rs 命令

检查 probe-rs 是否可用：

```bash
probe-rs --version
```

列出已连接的调试器：

```bash
probe-rs list
```

推荐使用 ELF 文件烧录：

```bash
probe-rs download --chip STM32F407IGHx --protocol swd --verify build/LED.elf
probe-rs reset --chip STM32F407IGHx --protocol swd
```

如果连接不稳定，可以降低 SWD 速度：

```bash
probe-rs download --chip STM32F407IGHx --protocol swd --speed 1000 --verify build/LED.elf
probe-rs reset --chip STM32F407IGHx --protocol swd --speed 1000
```

如果电脑上连接了多个调试器，先执行 `probe-rs list` 查看编号，再通过 `--probe` 指定：

```bash
probe-rs download --probe VID:PID:SERIAL --chip STM32F407IGHx --protocol swd --verify build/LED.elf
```

## 使用 bin 文件烧录

通常优先使用 `build/LED.elf`。只有在明确需要二进制文件时，才使用 `build/LED.bin`。STM32F407 的内部 Flash 起始地址为 `0x08000000`：

```bash
probe-rs download --chip STM32F407IGHx --protocol swd --binary-format bin --base-address 0x08000000 --verify build/LED.bin
probe-rs reset --chip STM32F407IGHx --protocol swd
```

## 清理后重新编译

如果怀疑构建缓存或旧产物影响烧录，可以先清理 CMake 目标，再重新编译：

```bash
cmake --build build --target clean
cmake --build build -j$(nproc)
```

如果 `build/` 配置已经损坏，可以删除后重新配置：

```bash
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

## 常见问题

### 找不到芯片或连接失败

先确认调试器和目标板连线：

- SWDIO
- SWCLK
- GND
- 目标板供电
- 可选 NRST

然后检查调试器是否被系统识别：

```bash
probe-rs list
```

如果仍然失败，尝试降低速度：

```bash
probe-rs download --chip STM32F407IGHx --protocol swd --speed 1000 --verify build/LED.elf
```

### 烧录成功但程序没有运行

烧录后手动复位：

```bash
probe-rs reset --chip STM32F407IGHx --protocol swd
```

同时确认当前烧录的是最新编译产物：

```bash
ls -l build/LED.elf build/LED.bin build/LED.hex
```

### 权限不足

如果普通用户无法访问调试器，通常是 udev 权限问题。可以临时使用管理员权限验证硬件链路：

```bash
sudo probe-rs list
```

长期使用时建议配置 probe-rs 或调试器对应的 udev 规则，避免每次使用 `sudo`。

## 备用 OpenOCD 烧录命令

当前推荐 probe-rs。若需要使用 OpenOCD，工程中保留了 `config/daplink.cfg` 和 `config/stlink.cfg`：

```bash
openocd -f config/daplink.cfg -c "program build/LED.elf verify reset exit"
```

ST-Link：

```bash
openocd -f config/stlink.cfg -c "program build/LED.elf verify reset exit"
```
