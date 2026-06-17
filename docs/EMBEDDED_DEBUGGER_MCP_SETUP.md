# 嵌入式调试器 MCP 安装配置 - RoboMaster C 板

## ✅ 安装完成

所有组件已于 2026-06-17 安装并验证完成。

## 已安装组件

### 1. probe-rs (v0.31.0)
- **位置**: `~/.cargo/bin/probe-rs`
- **用途**: 现代化 Rust 嵌入式调试工具集
- **验证状态**: ✅ 可烧录固件、读取内存、复位目标

### 2. embedded-debugger-mcp
- **位置**: `~/desktop/embedded-debugger-mcp/target/release/embedded-debugger-mcp`
- **用途**: 向 Claude 暴露 22 个调试工具的 MCP 服务器
- **状态**: ✅ 已注册到 Claude Code，健康检查通过

### 3. 硬件验证
- **调试探针**: DAPLink CMSIS-DAP (USB VID:PID = 0x0d28:0x0204)
- **目标板**: STM32F407IGHx RoboMaster C 板
- **连接**: ✅ SWD 工作正常，可读写内存

## 快速上手

### 烧录固件
```bash
probe-rs download cmake-build-debug/LED.elf --chip STM32F407IGHx --protocol swd --verify
```

### 读取内存（无需符号表）
```bash
probe-rs read b32 0x08000000 8 --chip STM32F407IGHx --protocol swd
```

### 复位目标
```bash
probe-rs reset --chip STM32F407IGHx --protocol swd
```

## 在 Claude 中使用 MCP 服务器

embedded-debugger MCP 服务器现已在此 Claude Code 会话中可用。你可以用自然语言：

**连接和检查：**
- "列出可用的调试探针"
- "连接到 STM32F407IGHx C 板"
- "从地址 0x08000000 读取 64 字节"
- "显示探针信息"

**Flash 操作：**
- "烧录 cmake-build-debug/LED.elf 到开发板"
- "验证已烧录的固件"
- "擦除整个 Flash 芯片"

**调试控制：**
- "暂停目标"
- "复位并运行固件"
- "单步执行 CPU"

**变量检查（需要带调试符号的 ELF）：**
- "读取 motor_chassis 数组"
- "显示 gyro 的值"
- "读取 rc_ctrl 结构体"

**RTT（实时传输）- 如果固件中启用：**
- "连接到 RTT"
- "从 RTT 终端通道读取"
- "向 RTT 下行通道发送 'hello'"

**断点：**
- "在 0x08000100 设置断点"
- "清除所有断点"

## 22 个可用 MCP 工具

### 探针管理
- `list_probes` - 发现连接的调试探针
- `connect` - 连接到目标芯片
- `probe_info` - 获取详细会话信息

### 内存操作
- `read_memory` - 读取 Flash/RAM（十六进制表、ihex、原始格式）
- `write_memory` - 写入 RAM 地址

### 调试控制
- `halt` - 停止 CPU 执行
- `run` - 恢复执行
- `reset` - 硬件/软件复位
- `step` - 单指令步进

### 断点
- `set_breakpoint` - 设置硬件/软件断点
- `clear_breakpoint` - 移除断点

### Flash 操作
- `flash_erase` - 擦除扇区或整个芯片
- `flash_program` - 编程 ELF/HEX/BIN 文件
- `flash_verify` - 验证 Flash 内容

### RTT 通信（6 个工具）
- `rtt_attach` - 连接到 RTT
- `rtt_detach` - 断开 RTT
- `rtt_channels` - 列出 RTT 通道
- `rtt_read` - 从 RTT 上行通道读取
- `rtt_write` - 写入 RTT 下行通道
- `run_firmware` - 一步完成部署 + RTT

### 会话管理
- `get_status` - 获取调试会话状态
- `disconnect` - 干净断开连接

## 架构对比

| 特性 | 旧脚本 (`scripts/debug/*.py`) | 新 MCP 服务器 |
|------|------------------------------|--------------|
| 连接方式 | OpenOCD + gdb-multiarch 子进程 | probe-rs 原生（直连 USB）|
| 数据格式 | 正则表达式解析 GDB stdout | 结构化 JSON |
| 状态 | 无状态（每次新进程） | 有状态会话 |
| 符号解析 | GDB DWARF 解析器 | probe-rs ELF 解析器 |
| RTT | 不可用 | 双向、实时 |
| 速度 | ~500ms+ 每次查询（进程启动） | ~10ms（内存会话）|
| Agent 友好性 | ❌ 文本抓取脆弱 | ✅ 结构化工具调用 |

## 下一步

1. **立即尝试**: 对我说 "连接到 STM32 开发板并从 0x08000000 读取 32 字节"
2. **诊断 BMI088 卡死**: "暂停目标并显示 PC 寄存器和调用栈"
3. **监控实时数据**: 在固件中添加 RTT → 无需 UART 的实时 printf
4. **与 OpenOCD 对比**: 你现有的 `config/daplink.cfg` 仍可用于 GDB 调试

## 重要提示

⚠️ **探针独占性**: 同一时间只有一个工具能访问 DAPLink：
- 如果运行了 `openocd -f config/daplink.cfg`，MCP 服务器无法连接
- 如果 MCP 有打开的会话，openocd 会失败
- 使用前务必先断开/终止其中一个

⚠️ **二进制位置**: MCP 二进制现在在 `~/desktop/embedded-debugger-mcp/` - 重启后不会丢失。

## 故障排除

**访问探针时"权限被拒绝"：**
```bash
# 安装 udev 规则（已完成，但如有问题）：
curl -fsSL https://probe.rs/files/69-probe-rs.rules | sudo tee /etc/udev/rules.d/69-probe-rs.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
# 重新插拔探针
```

**MCP 服务器无响应：**
```bash
# 检查 MCP 健康状态
claude mcp list

# 查看日志（如果设置了 RUST_LOG=debug）
# 日志输出到 stderr，在 Claude 工具执行输出中可见
```

**符号解析失败：**
- 确保 `cmake-build-debug/LED.elf` 包含调试符号：`file cmake-build-debug/LED.elf` 应显示 "not stripped"
- 如需要，使用 `-g` 标志重新编译

## 参考资料

- [probe-rs 文档](https://probe.rs/)
- [embedded-debugger-mcp GitHub](https://github.com/Adancurusul/embedded-debugger-mcp)
- [STM32F407 参考手册](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)

---

**安装完成时间**: 2026-06-17 13:25 UTC
**硬件**: STM32F407IGHx + DAPLink CMSIS-DAP
**固件**: `/home/rick/desktop/RM-C-board-volleyball/cmake-build-debug/LED.elf`
