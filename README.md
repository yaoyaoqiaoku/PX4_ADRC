# PX4 ADRC 角速率控制器

> 基于 LADRC（线性自抗扰控制）的 PX4 多旋翼角速率内环替换模块

[![PX4 v1.15.4](https://img.shields.io/badge/PX4-v1.15.4-blue.svg)](https://github.com/PX4/PX4-Autopilot/tree/v1.15.4)
[![License](https://img.shields.io/badge/license-BSD--3-green.svg)](LICENSE)

## 版本

| 标签 | 说明 |
|---|---|
| [`adrc-v1.0`](https://github.com/yaoyaoqiaoku/PX4_ADRC/tree/adrc-v1.0) | 基础版：LADRC + 增广 ESO + 分配器反馈 + γ 补偿 |
| [`adrc-v2.0`](https://github.com/yaoyaoqiaoku/PX4_ADRC/tree/adrc-v2.0) | **推荐**：v1.0 + lambda_z3 + dt_eso 保护 + motor_tau 精确离散化 + AF 自适应滤波 + keep-last-valid 模式校验 + landed-only 重置 |

```bash
git checkout adrc-v1.0   # 切到旧版
git checkout adrc-v2.0   # 切到新版（推荐）
```

## 项目概述

用 ADRC（Active Disturbance Rejection Control）替换 PX4 原版 PID 角速率控制器 `mc_rate_control`，保留接口不变，只换控制律。

**核心思想**：ESO（扩张状态观测器）实时估计"总扰动"（包括未建模动态、外部扰动、模型误差），控制器主动补偿——在重载、大风、挂载变化等场景下比 PID 更鲁棒。

**验证状态**：
- ✅ SITL（Gazebo Classic / gz-sim）通过
- ✅ 真机验证：PX4_FMU_V6C / 66cm 轴距 / 3.5kg / U3515 电机 / 14 寸桨 / 6S
- ✅ 26 轮真机扫参验证，推荐参数 Roll 5.4°/s、Pitch 5.0°/s（达到 PID 优秀水平）
- ✅ CW 扫参：CW=10 是机架天花板，CW=12~15 平静期振动 3.9~4.4°/s（与 CW=10 的 3.1°/s 肉眼不可分辨）
- ⏳ 抗扰对比测试（风场/挂载变化）待完成

## 目录结构

```
PX4_ADRC/
├── src/modules/adrc_rate_control/       # ADRC 模块源码
│   ├── Adrc.hpp                         # 算法类声明
│   ├── Adrc.cpp                         # 算法实现（LADRC + 增广 ESO + 饱和建模）
│   ├── AdrcRateControl.hpp              # PX4 模块声明
│   ├── AdrcRateControl.cpp              # PX4 模块实现（订阅/发布/调度/安全逻辑）
│   ├── adrc_params.c                    # 91 个参数定义
│   ├── CMakeLists.txt                   # 构建接入
│   └── Kconfig                          # 模块开关
├── msg/
│   └── AdrcStatus.msg                   # 调试话题定义（z1/z2/z3/v1/v2/u/e1/y/ueso/integ/z3raw/sp）
├── docs/
│   ├── ADRC真机优化-底层改进方案.md      # 完整设计文档（v1~v4 全部迭代记录）
│   ├── ADRC真机调试测试指南.md           # 真机调参 step-by-step 指南
│   ├── ADRC替换PX4-学习笔记.md          # 学习笔记（PX4 参数系统、模块接入）
│   ├── PX4-ADRC内环替换-阶段二设计说明.md # 阶段二设计文档
│   ├── ADRC参数仿真测试指南.md           # 控制器级仿真测试（注入真机特征）
│   ├── ADRC开源项目审计与AP_ADRC溯源.md  # 开源参考项目审计
│   ├── Gazebo加风与ADRC抗扰测试指南.md   # SITL 风场测试方法
│   ├── params_adrc_real.txt             # 真机参数预设（可直接粘贴）
│   ├── params_from_log11.txt            # SITL 定版参数（完整 1003 个）
│   ├── calibrate_b0.py                  # b0 标定脚本 v1
│   ├── calibrate_b0_v2.py               # b0 标定脚本 v2（支持闭环数据）
│   ├── analyze_logs.py                  # 日志分析脚本（角速率/FFT/z3/u）
│   ├── verify_213.py                    # 姿态环驱动验证脚本
│   ├── gust_inject.py                   # SITL 阵风注入工具（Gazebo wrench）
│   ├── seq_adrc_basic.csv               # 标准抗扰测试序列
│   ├── adrc_v3_param_sim_trace.png      # 参数仿真时域对比图
│   ├── adrc_v3_param_sim_notch_warning.png # Notch 相位滞后警告图
│   ├── adrc_v3_sitl_hover_log12.png     # SITL 悬停段可视化
│   └── log213_diagnosis.png             # 真机 log_213 诊断图
└── README.md
```

## 架构

### 替换点

```
位置环 mc_pos_control → 姿态环 mc_att_control → [角速率环] → 控制分配 → 电机
                              ↑ 替换这里
原版：mc_rate_control (PID)
新版：adrc_rate_control (LADRC)
```

接口完全一致（订阅 `vehicle_rates_setpoint` + `vehicle_angular_velocity`，发布 `vehicle_torque_setpoint` + `vehicle_thrust_setpoint`），姿态环、控制分配、模式管理全部不用动。

### 数据流

```
mc_att_control ── vehicle_rates_setpoint ──┐
EKF2 ── vehicle_angular_velocity ──────────┤
EKF2 ── vehicle_angular_acceleration ──────┼──> adrc_rate_control ──> vehicle_torque_setpoint ──> 控制分配
land_detector ── vehicle_land_detected ────┤         │
commander ── vehicle_control_mode ─────────┘         └──> adrc_status（调试）
```

### 算法结构（每轴一套）

**LADRC 模式（默认，ADRC_ESO_MODE=1）**：

```
                    ┌─────────────┐
  sp ──────────────>│  设定值平滑  │──> sp_smooth
                    │  (SPS LPF)  │
                    └─────────────┘
                           │
                           v
                    ┌─────────────┐
  e1 = sp - z1 ───>│  u0 = wc*e1 │──> u = (u0 - γ*z3) / b0 ──> 归一化力矩
                    │  + FF*sp    │
                    │  + KI*∫e1   │
                    └─────────────┘
                           │
                           v
                    ┌─────────────┐
  y (反馈) ────────>│    ESO      │──> z1 (角速率估计)
  u (控制) ────────>│  (线性/增广) │──> z2 (执行器状态，增广模式)
                    │             │──> z3 (总扰动估计)
                    └─────────────┘
```

**增广 ESO（TAU > 0 且 3·ESO_W·TAU > 1 时自动激活）**：

```
植物模型:  τ·ẋ_a = u - x_a        (执行器一阶滞后)
           ω̇    = b0·x_a + f      (f = 总扰动)

观测器:    ż1 = b0·z2 + z3 - β1·e   (z1 = 角速率)
           ż2 = (u - z2)/τ - β2·e   (z2 = 执行器扭矩状态，物理约束 [-1,1])
           ż3 = -β3·e               (z3 = 总扰动)

增益:      β1 = 3ωo - 1/τ,  β2 = (3ωo² - β1/τ - τωo³)/b0,  β3 = τωo³
           (极点精确配置到 -ωo)
```

## 集成到 PX4 v1.15.4

### 前置条件

- PX4 v1.15.4 源码（`git clone --branch v1.15.4 https://github.com/PX4/PX4-Autopilot.git`）
- CMake ≥ 3.20
- ARM 工具链（真机）或 GCC（SITL）

### 步骤 1：复制模块文件

```bash
cd PX4-Autopilot

# 复制 ADRC 模块
cp -r <this-repo>/src/modules/adrc_rate_control src/modules/

# 复制消息定义
cp <this-repo>/msg/AdrcStatus.msg msg/
```

### 步骤 2：注册消息

编辑 `msg/CMakeLists.txt`，在 `set(msg_files ...)` 列表中添加：

```
AdrcStatus.msg
```

### 步骤 3：启用模块

编辑目标板级配置文件：

**SITL**（`boards/px4/sitl/default.px4board`）：
```diff
- CONFIG_MODULES_MC_RATE_CONTROL=y
+ CONFIG_MODULES_ADRC_RATE_CONTROL=y
```

**真机 fmu-v6c**（`boards/px4/fmu-v6c/default.px4board`）：
```diff
- CONFIG_MODULES_MC_RATE_CONTROL=y
+ CONFIG_MODULES_ADRC_RATE_CONTROL=y
```

**注意**：如果需要保留 PID 回退能力，**不要删除** `CONFIG_MODULES_MC_RATE_CONTROL=y`，而是同时启用两个模块，然后通过启动脚本选择。

### 步骤 4：修改启动脚本

编辑 `ROMFS/px4fmu_common/init.d/rc.mc_apps`：

```diff
 # Start Multicopter Rate Controller.
 #
-mc_rate_control start
+adrc_rate_control start
```

编辑 `ROMFS/px4fmu_common/init.d/CMakeLists.txt`：

```diff
-if(CONFIG_MODULES_MC_RATE_CONTROL)
+if(CONFIG_MODULES_MC_RATE_CONTROL OR CONFIG_MODULES_ADRC_RATE_CONTROL)
```

### 步骤 5：添加日志话题（推荐）

编辑 `src/modules/logger/logged_topics.cpp`，在适当位置添加：

```cpp
add_optional_topic("adrc_status", 5);  // 5ms 间隔 = 200Hz
```

### 步骤 6：编译

```bash
# SITL
make px4_sitl_default

# 真机 fmu-v6c
make px4_fmu-v6c_default

# 真机 micoair h743-v2
make micoair_h743-v2_default
```

### 步骤 7：设置参数

烧录后在 QGC 参数界面或 MAVLink Console 设置参数。详见 `docs/ADRC真机调试测试指南.md`。

## 参数速查表

共 91 个参数，按功能分组：

### 模式切换

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `ADRC_ESO_MODE` | INT32 | 1 | 0=经典 fal ESO，1=线性 ESO（LADRC，默认） |
| `ADRC_CTRL_LAW` | INT32 | 2 | 0=非线性 fal 律，2=线性律（模式 0 用） |

### 核心参数（每轴）

| 参数 | 含义 | 默认 | 真机推荐 | 说明 |
|---|---|---|---|---|
| `ADRC_*_CW` | 控制带宽 ωc (rad/s) | 11 | 8 | 速率环带宽，约 PID 增益的 1/2~2/3 |
| `ADRC_*_ESO_W` | 观测器带宽 ωo (rad/s) | 50 | 30 | 扰动估计速度，需 3·ESO_W·TAU>1 激活增广 ESO |
| `ADRC_*_B0` | 控制增益 | 100 | 按机架标定 | 物理意义：归一化扭矩→角加速度增益 |

### 硬化参数（每轴）

| 参数 | 含义 | 默认 | 真机推荐 | 说明 |
|---|---|---|---|---|
| `ADRC_*_TAU` | 执行器延时 (s) | 0 | 0.03 | 14 寸桨典型 0.02~0.03，激活增广 ESO |
| `ADRC_*_GAMMA` | 扰动补偿系数 | 1.0 | 0.7 | 1=全补偿，<1=部分补偿（更鲁棒） |
| `ADRC_*_FLT` | 反馈低通 (Hz) | 0 | 30 | 压陀螺噪声进 ESO |
| `ADRC_*_NF` | 反馈陷波 (Hz) | 0 | 0 | 对准谐振峰（最后手段） |
| `ADRC_*_NBW` | 陷波带宽 (Hz) | 1 | 1 | |
| `ADRC_*_Z3MAX` | z3 限幅 (rad/s²) | 0 | 0 | 0=自动 2·b0 |
| `ADRC_*_FF` | 前馈增益 | 0 | 0~0.6 | 低带宽下恢复跟手，需配 KI |
| `ADRC_*_KI` | 积分增益 | 0 | 0~0.3 | 消除静差，带抗饱和 |
| `ADRC_*_RAMP` | 输出限速 (1/s) | 0 | 0~20 | 防阶跃激励柔性模态 |
| `ADRC_*_SPS` | 设定值平滑 (Hz) | 0 | 0~10 | 打杆就晃时开 |
| `ADRC_*_LZ3` | z3 漏积分衰减率 | 0 | 2 | 防 z3 慢漂，Yaw 建议 0（保留常值扰动） |
| `ADRC_*_AF` | 自适应滤波阈值 (rad/s²) | 0 | 0~8 | 包络自适应低通，阈值需 >z3 正常波动幅度 |

### 非线性 ESO 参数（模式 0 用）

| 参数 | 含义 | 默认 |
|---|---|---|
| `ADRC_*_R` | TD 快速跟踪因子 | 100/100/50 |
| `ADRC_*_DELTA` | fal() 线性区间宽度 | 0.015 |
| `ADRC_*_B01~B03` | ESO 增益 | 150/250/550 |
| `ADRC_*_NB1, NB2` | NLSEF 增益 | 10/0.001 |
| `ADRC_*_A1, A2` | NLSEF 指数 | 0.9/1.5 |

### 调试话题

`adrc_status` 话题字段：

| 字段 | 含义 |
|---|---|
| `z1[3]` | ESO 角速率估计 |
| `z2[3]` | ESO 第二状态（增广模式=执行器扭矩，非线性模式=角加速度估计） |
| `z3[3]` | ESO 总扰动估计 |
| `v1[3]`, `v2[3]` | TD 参考及其微分 |
| `u[3]` | 控制输出（clamp 后） |
| `e1[3]`, `e2[3]` | 控制误差 |
| `y[3]` | 经滤波后喂给 ESO 的反馈 |
| `ueso[3]` | 增广模式=执行器状态估计，非线性模式=滞后滤波输出 |
| `integ[3]` | LADRC 积分状态 |
| `z3raw[3]` | z3 限幅前原始值 |
| `sp[3]` | 实际送入控制器的设定值 |

## 调参指南

### 调参原则

1. **每轮只改一组参数**，改完 `param save`，飞一次验证
2. **先稳后快**：先让飞机稳定悬停，再逐步提升带宽
3. **TAU 优先于降 ESO_W**：真机极限环的主因是执行器延时被 ESO 当扰动，升 TAU（更准确的延时模型）比降 ESO_W（降低观测器速度）更对症
4. **γ 是鲁棒性旋钮**：γ=1 全补偿（理论最优），γ=0.7 实战最优（牺牲一点抗扰换稳定性）

### 真机起点参数（66cm/3.5kg/14 寸桨/6S）

```bash
# 核心（v2.0 推荐参数，26 轮真机扫参验证）
param set ADRC_ROLL_CW 10
param set ADRC_PITCH_CW 10
param set ADRC_YAW_CW 5
param set ADRC_ROLL_ESO_W 40
param set ADRC_PITCH_ESO_W 40
param set ADRC_YAW_ESO_W 15
param set ADRC_ROLL_B0 100
param set ADRC_PITCH_B0 100
param set ADRC_YAW_B0 20

# 硬化
param set ADRC_ROLL_TAU 0.035
param set ADRC_PITCH_TAU 0.035
param set ADRC_YAW_TAU 0.01
param set ADRC_ROLL_GAMMA 0.7
param set ADRC_PITCH_GAMMA 0.7
param set ADRC_YAW_GAMMA 0.6
param set ADRC_ROLL_FLT 30
param set ADRC_PITCH_FLT 30
param set ADRC_YAW_FLT 20

# v2.0 新增
param set ADRC_ROLL_LZ3 2
param set ADRC_PITCH_LZ3 2
param set ADRC_YAW_LZ3 0        # Yaw 不开漏积分（保留常值扰动补偿）
param set ADRC_ROLL_AF 0         # AF 关闭（阈值过低反而有害）
param set ADRC_PITCH_AF 0
param set ADRC_YAW_AF 0

# 姿态环
param set MC_ROLL_P 4.5
param set MC_PITCH_P 4.5
param set MC_YAW_P 4.0
param set MC_AIRMODE 0

param save
```

### QGC 一键导入

仓库根目录 `params/adrc_params_recommended.params` 可在 QGC 中直接导入（设置 → 参数 → 工具 → Load from file）。

### 调参流程

```
起点（CW 10, ESO_W 40, TAU 0.035, γ 0.7, FLT 30, LZ3 2, AF 0, b0 100, P_att 4.5）
  │
  ├─ 悬停 30s ──→ 稳定？
  │     │              │
  │     │ 否           │ 是
  │     ▼              ▼
  │  1~2Hz 慢摇     CW → 11 → 12（注意 CW=12~13 可能踩 Roll 共振带）
  │     │              │
  │     ▼              ▼
  │  ↑CW / ↓P_att   3Hz 振动？
  │                    │
  │                    ▼
  │              ↓γ (0.7→0.6) / ↑TAU (0.035→0.04)
  │                    │
  │                    ▼
  │              b0 标定（calibrate_b0_v2.py）
  │                    │
  │                    ▼
  │              抗扰测试（gust_inject.py）
  │
  ├─ 2~4Hz 高频抖
  │     │
  │     ▼
  │  ↑TAU (0.03→0.035→0.04)
  │  ↓γ (0.7→0.6)
  │  不要 ↓CW
  │
  └─ z3 发散
        │
        ▼
     ↓ESO_W / ↓γ / 标定 b0
```

### 故障处理

| 现象 | 原因 | 处理 |
|---|---|---|
| 1~2Hz 慢摇 | 内环太软 | ↑CW / ↓MC_ROLL_P |
| 2~4Hz 高频抖 | ESO 与执行器谐振 | ↑TAU / ↓γ（不要 ↓CW） |
| 离地就晃 | 有效增益不足 | ↑CW / 确认 b0 / 满电电池 |
| z3 疯狂翻转 | ESO 发散 | ↓ESO_W / ↓γ / 标定 b0 |
| 打杆就炸 | 阶跃激励柔性模态 | 开 SPS / 开 RAMP |

## b0 标定

b0 是"归一化扭矩 → 角加速度"的增益，依赖机架惯量、电机推力、电池电压。

### 物理计算

```
b0_roll/pitch = 2 × T_max × a / J_roll

其中：
  T_max = 电机最大推力（查数据表或推力台实测）
  a = 半臂长（66cm 轴距 → a = 0.233m）
  J_roll = roll 轴转动惯量（CAD 或摆荡法：J = m·g·a·T²/(4π²)）
```

### 飞行标定

```bash
# 使用 v2 脚本（支持闭环数据）
python3 docs/calibrate_b0_v2.py <your_log.ulg> --b0-used 100 100 8
```

或 Acro 模式力矩阶跃测试（最精确）：
1. 起飞，切 Acro 模式
2. 快速推 roll 杆 20~30%，保持 0.3 秒，松开
3. 重复 3~5 次
4. 用脚本提取 b0 = Δω̇ / Δu

详见 `docs/ADRC真机调试测试指南.md` §7。

## SITL 仿真

### 快速启动

```bash
cd PX4-Autopilot
make px4_sitl gz_x500
```

机架脚本 `4001_gz_x500` 已预设稳定参数（ESO_W=25/30/30, CW=7.5/8/7, γ=0.7）。

### 阵风测试

```bash
# 自检
python3 docs/gust_inject.py --check

# 8N 侧风阶跃 3 秒
python3 docs/gust_inject.py --force 8,0,0 --wave step --duration 3

# 标准测试序列（38 秒，包含阶跃/脉冲/持续侧风）
python3 docs/gust_inject.py --seq docs/seq_adrc_basic.csv
```

详见 `docs/Gazebo加风与ADRC抗扰测试指南.md`。

## 日志分析

```bash
# 快速分析（角速率 RMS / FFT / z3 / u）
python3 docs/analyze_logs.py <your_log.ulg>

# 姿态环驱动验证（检查是否是级联失稳）
python3 docs/verify_213.py

# b0 标定
python3 docs/calibrate_b0_v2.py <your_log.ulg> --b0-used 100 100 8
```

## 设计文档

| 文档 | 内容 |
|---|---|
| `ADRC真机优化-底层改进方案.md` | 完整设计文档：v1 基础版 → v2 硬化项 → v3 γ/SPS → v4 增广 ESO + 饱和建模，含根因分析、参考项目、参数速查 |
| `ADRC真机调试测试指南.md` | 真机调参 step-by-step：前置条件、安全守则、PID 基线、逐步调参、故障处理、b0 标定 |
| `PX4-ADRC内环替换-阶段二设计说明.md` | 阶段二设计：接口设计、算法设计、参数清单、状态管理、验证计划 |
| `ADRC替换PX4-学习笔记.md` | 学习笔记：PX4 参数系统、模块接入、调参回顾 |
| `ADRC参数仿真测试指南.md` | 控制器级仿真：注入真机特征（噪声/延时/柔性模态），逐个参数测试 |
| `ADRC开源项目审计与AP_ADRC溯源.md` | 开源参考：FMT/AP_ADRC/ThisisADRC/ACFLY 等项目的源码分析 |
| `Gazebo加风与ADRC抗扰测试指南.md` | SITL 风场测试：恒定风/wrench 注入/Gazebo Classic 完整风场 |

## 已知限制

1. **PID 回退**：当前 `rc.mc_apps` 无条件启动 `adrc_rate_control`，原版 `mc_rate_control` 已从板级配置移除。如需保留 PID 回退能力，需同时编译两个模块并实现 `RATE_CTRL_MODE` 参数切换。
2. **悬停 b0 标定精度**：悬停数据激励不足，所有闭环辨识方法都有较大不确定性（±30%）。精确标定需 Acro 模式力矩阶跃测试。
3. **低频 Notch 风险**：`ADRC_*_NF` 在临界工况的相位滞后可能比谐振本身更危险，仅作最后手段。
4. **电池缩放**：若启用 `MC_BAT_SCALE_EN`，ESO 输入与实际扭矩存在缩放偏差（当前未处理）。
5. **CW 上限**：66cm/14寸桨机架 CW=12~13 可能触发 Roll 轴结构共振带（平静期 z1 从 3°/s 跳到 4°/s，肉眼不可见但数据可测），CW=10 是该机架的保守上限。
6. **AF 阈值**：AF 的包络阈值需高于 z3 正常波动幅度（该机架约 10~15 rad/s²），AF=8 过低会导致持续重滤波、延迟补偿，反而恶化振动。建议关闭（AF=0）。

## 更新日志

### v2.0（2026-08-21）

**新增功能**：
- `ADRC_*_LZ3`：z3 漏积分衰减率，防陀螺零偏导致 z3 慢漂（Yaw 轴建议关闭）
- `ADRC_*_AF`：自适应扰动滤波，包络跟踪 + 立方带宽衰减（阈值需匹配 z3 波动幅度）
- dt_eso 稳定性保护：线性 ESO 用 `min(h, 1/ωo)`，非线性 ESO 用 `min(h, 2/max_gain)`
- motor_tau 精确离散化：`alpha = 1 - exp(-h/tau)`（ZOH，无条件稳定）
- NaN 防护：`update()` 入口 PX4_ISFINITE 三连检
- z3_filt 防御性钳位：滤波后 z3 不超过物理极限
- keep-last-valid 模式校验：非法 ADRC_ESO_MODE/CTRL_LAW 保持上一有效模式 + 警告
- 合法模式切换自动重置三轴状态（z1 以当前速率播种）
- resetIntegral() 清空完善（z3/z2/u_eso/z3_filt，防起飞跳变）

**改进**：
- landed-only 重置：移除 maybe_landed（防飞行中误判清零导致扰动补偿丢失）
- 推荐参数更新：CW=10, ESO_W=40, TAU=0.035（基于 26 轮真机扫参验证）

### v1.0（初始版本）

- 基础 LADRC + 增广 ESO + 分配器饱和反馈 + γ 部分补偿
- 全套硬化：FLT/NF/NBW/SPS/RAMP/KI/ILIM/Z3MAX
- 74 个参数，per-axis 独立

## 参考资料

| 项目 | 参考价值 |
|---|---|
| [FMT ADRC](https://github.com/Firmament-Autopilot/FMT-Firmware) | 阿木实验室真机验证的 ADRC，γ 补偿系数来源 |
| [Jiachi Zou 论文](https://pure.tue.nl/ws/portalfiles/portal/110035542/Jiachi_Zou_Thesis.pdf) | LADRC 理论基础，γ 补偿系数的理论依据 |
| [ArduPilot AP_ADRC PR #20243](https://github.com/ArduPilot/ardupilot/pull/20243) | ArduPilot 社区 ADRC 尝试（Draft，未合并） |
| [ThisisADRC](https://github.com/zzqzzqzzq2002/ThisisADRC) | ESO 状态饱和、NLSEF 积分项参考 |
| [zhaohaojie1998/Control-Algorithm](https://github.com/zhaohaojie1998/Control-Algorithm) | 本模块的算法原型 |
| [StarryPilot adrc_att.c](https://github.com/JcZou/StarryPilot) | JcZou 真机验证的 ADRC 实现 |

## 许可证

BSD-3-Clause（与 PX4 一致）。
