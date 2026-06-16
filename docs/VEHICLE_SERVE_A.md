# 发球车A配置

## 车辆信息

- **车辆名称**: 发球车A (Serve Vehicle A)
- **分支名称**: `serve-vehicle-A`
- **创建日期**: 2026-06-16
- **基础分支**: `master` (commit c48aa75)

## 硬件配置

### 底盘系统
- **电机类型**: M3508 x4
- **底盘结构**: 麦克纳姆轮全向底盘
- **CAN 总线**: CAN1
- **电机 ID**: 
  - Motor1: 0x201 (右前)
  - Motor2: 0x202 (左前)
  - Motor3: 0x203 (右后)
  - Motor4: 0x204 (左后)

### 传感器
- **IMU**: BMI088 六轴 IMU (SPI1)
  - 加速度计
  - 陀螺仪
  - 航向保持功能

### 控制接口
- **遥控器**: SBUS (USART3)
- **上位机通信**: UART (USART1, 115200bps)

### 移除的硬件（相比主车）
- ❌ DM4340 执行电机 (CAN2) - 本车不使用
- ❌ 红外触发 GPIO (PB12) - 本车不使用

## 软件配置

### 控制模式

| 模式 | 遥控器右拨杆 | 说明 |
|------|-------------|------|
| type=3 | 中档 | 失能模式，所有电机停止 |
| type=1 | 上拨 | 遥控模式，摇杆控制底盘全向移动 |
| type=0 | 下拨 | IMU 校准模式（2秒后自动进入 type=2） |
| type=2 | 自动 | 自动追球 + 遥控融合模式 |

### 功能状态

#### 启用的功能 ✅
- [x] M3508 麦克纳姆轮底盘控制
- [x] BMI088 IMU 姿态感知
- [x] 卡尔曼滤波（加速度计零偏估计）
- [x] 陀螺仪航向保持（Z 轴 PID 回正）
- [x] SBUS 遥控器控制
- [x] 串口接收上位机坐标
- [x] UART DMA 发送队列
- [x] 3 种控制模式切换

#### 禁用的功能 ❌
- [ ] DM4340 电机控制（CAN2 不使用）
- [ ] GPIO 中断触发（PB12 不使用）
- [ ] 执行机构控制

## 代码修改需求

### 待修改项
1. **禁用 DM4340 相关代码**
   - 注释掉 `DM4340_Control_Init()`
   - 注释掉 `CAN_cmd_motor_control()`
   - 注释掉 `DM4340_Control_Loop()`
   - 注释掉 `DM4340_Set_Target_Angle()`

2. **禁用 GPIO 中断代码**
   - 注释掉 PB12 中断处理
   - 移除相关的电机动作序列

3. **简化控制逻辑**
   - 保留底盘控制
   - 保留 IMU 和遥控器
   - 移除执行机构相关状态

### 配置参数

#### PID 参数（底盘电机）
```c
// 底盘电机 PID: Kp=3.5, Ki=0.01, Kd=0.0
pid_init(&motor1_pid, 3.5, 0.01, 0.0, 30000, 16384);
pid_init(&motor2_pid, 3.5, 0.01, 0.0, 30000, 16384);
pid_init(&motor3_pid, 3.5, 0.01, 0.0, 30000, 16384);
pid_init(&motor4_pid, 3.5, 0.01, 0.0, 30000, 16384);
```

#### 航向 PID 参数
```c
// Wz 航向保持 PID: Kp=1.5, Ki=0.0, Kd=0.0
pid_init(&wz_pid, 1.5, 0.00, 0.0, 500, 660);
```

## 调试记录

### 2026-06-16 初始化
- 创建分支 `serve-vehicle-A`
- 从 master 分支 (c48aa75) 创建
- 基础硬件：4个M3508 + BMI088 + SBUS遥控

### 已知问题
- 待测试所有 4 个 M3508 电机连接状态
- 待验证遥控器通信
- 待测试 IMU 校准功能

## 测试清单

### 硬件测试
- [ ] M3508 Motor1 (0x201) 通信测试
- [ ] M3508 Motor2 (0x202) 通信测试
- [ ] M3508 Motor3 (0x203) 通信测试
- [ ] M3508 Motor4 (0x204) 通信测试
- [ ] BMI088 IMU 数据读取
- [ ] SBUS 遥控器接收
- [ ] 上位机串口通信

### 功能测试
- [ ] 遥控模式 (type=1) 底盘控制
- [ ] IMU 校准 (type=0)
- [ ] 自动追球模式 (type=2)
- [ ] 航向保持功能
- [ ] 麦克纳姆轮全向移动
- [ ] 紧急停止功能

### 性能测试
- [ ] 底盘响应速度
- [ ] PID 控制稳定性
- [ ] IMU 数据更新频率
- [ ] 控制循环频率

## 快速命令

### 编译烧录
```bash
cd cmake-build-debug
make -j$(nproc)
openocd -f ../config/daplink.cfg -c "program LED.elf verify reset exit"
```

### 测试 CAN 通信
```bash
openocd -f config/daplink.cfg &
python3 scripts/debug/auto_test_can.py
```

### 切换到此分支
```bash
git checkout serve-vehicle-A
```

## 维护记录

| 日期 | 修改内容 | 备注 |
|------|---------|------|
| 2026-06-16 | 创建分支和配置文档 | 初始化发球车A |

## 相关文档

- [../README.md](../README.md) - 项目主文档
- [../docs/DEBUG.md](../docs/DEBUG.md) - 调试指南
- [../CLAUDE.md](../CLAUDE.md) - 快速开发指南
