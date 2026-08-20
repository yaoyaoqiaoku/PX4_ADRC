# PX4 ADRC 角速率控制器

基于 LADRC（线性自抗扰控制）的 PX4 多旋翼角速率内环替换模块，替代原版 PID `mc_rate_control`。

## 项目概述

- **基线**：PX4 v1.15.4
- **飞控板**：PX4_FMU_V6C（真机）、SITL（仿真）
- **真机验证**：66cm 轴距 / 3.5kg / U3515 电机 / 14 寸桨 / 6S
- **状态**：SITL 通过，真机已验证可稳定悬停

## 目录结构

```
├── src/modules/adrc_rate_control/   # ADRC 模块源码
│   ├── Adrc.hpp / Adrc.cpp          # 算法核心（LADRC + 增广 ESO）
│   ├── AdrcRateControl.hpp / .cpp   # PX4 模块外壳
│   ├── adrc_params.c                # 74 个参数定义
│   ├── CMakeLists.txt / Kconfig     # 构建接入
├── msg/AdrcStatus.msg               # 调试话题定义
└── docs/                            # 文档与工具
    ├── ADRC真机优化-底层改进方案.md   # 完整设计文档
    ├── ADRC真机调试测试指南.md        # 真机调参指南
    ├── params_adrc_real.txt          # 真机参数预设
    ├── calibrate_b0_v2.py            # b0 标定脚本
    ├── gust_inject.py                # SITL 阵风注入工具
    └── seq_adrc_basic.csv            # 标准测试序列
```

## 集成到 PX4 v1.15.4

### 1. 克隆 PX4 v1.15.4

```bash
git clone --branch v1.15.4 --depth 1 https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git submodule update --init --recursive
```

### 2. 复制 ADRC 模块

```bash
# 从本仓库复制
cp -r <this-repo>/src/modules/adrc_rate_control src/modules/
cp <this-repo>/msg/AdrcStatus.msg msg/
```

### 3. 注册消息

编辑 `msg/CMakeLists.txt`，在 `set(msg_files ...)` 列表中添加：

```
AdrcStatus.msg
```

### 4. 启用模块

编辑目标板级配置（如 `boards/px4/sitl/default.px4board` 或 `boards/px4/fmu-v6c/default.px4board`）：

```diff
- CONFIG_MODULES_MC_RATE_CONTROL=y
+ CONFIG_MODULES_ADRC_RATE_CONTROL=y
```

### 5. 修改启动脚本

编辑 `ROMFS/px4fmu_common/init.d/rc.mc_apps`：

```diff
- mc_rate_control start
+ adrc_rate_control start
```

编辑 `ROMFS/px4fmu_common/init.d/CMakeLists.txt`，将条件改为：

```diff
- if(CONFIG_MODULES_MC_RATE_CONTROL)
+ if(CONFIG_MODULES_MC_RATE_CONTROL OR CONFIG_MODULES_ADRC_RATE_CONTROL)
```

### 6. 添加日志话题（可选）

编辑 `src/modules/logger/logged_topics.cpp`，在适当位置添加：

```cpp
add_optional_topic("adrc_status", 5);  // 5ms = 200Hz
```

### 7. 编译

```bash
# SITL
make px4_sitl_default

# 真机（以 fmu-v6c 为例）
make px4_fmu-v6c_default
```

## 快速调参

详见 `docs/ADRC真机调试测试指南.md`。核心参数：

| 参数 | 含义 | 推荐起点 |
|---|---|---|
| `ADRC_*_CW` | 控制带宽 (rad/s) | 8 |
| `ADRC_*_ESO_W` | 观测器带宽 (rad/s) | 30 |
| `ADRC_*_B0` | 控制增益 | 按机架标定 |
| `ADRC_*_TAU` | 执行器延时 (s) | 0.03 |
| `ADRC_*_GAMMA` | 扰动补偿系数 | 0.7 |
| `ADRC_*_FLT` | 反馈低通 (Hz) | 30 |

## 关键设计

1. **LADRC**（默认）：线性 ESO + 带宽参数化，只需调 ωo 和 ωc 两个核心旋钮
2. **三阶增广 ESO**：TAU>0 时自动激活，执行器延时建模为观测器状态，防止 ESO 把延时当扰动补偿
3. **分配器饱和反馈**：镜像原版 PID 的反饱和，ESO 吃实际达成扭矩而非指令
4. **扰动补偿系数 γ**：γ<1 降低补偿强度，提高真机鲁棒性（FMT / Jiachi Zou 论文）
5. **A/B 回退**：通过 `RATE_CTRL_MODE` 参数可切回原版 PID（需同时编译 mc_rate_control）

## 参考资料

- [FMT ADRC](https://github.com/Firmament-Autopilot/FMT-Firmware) — 阿木实验室真机验证的 ADRC 实现
- [Jiachi Zou 论文](https://pure.tue.nl/ws/portalfiles/portal/110035542/Jiachi_Zou_Thesis.pdf) — LADRC 理论基础
- [ArduPilot AP_ADRC PR #20243](https://github.com/ArduPilot/ardupilot/pull/20243) — ArduPilot 社区 ADRC 尝试
