# PX4 角速率内环 ADRC 替换 — 阶段二设计说明

> 版本：v0.3（已实现并通过 SITL 编译；实际目标版本为 v1.15.4，用户工作树）
> 日期：2026-08-10
> 关联仓库：zhaohaojie1998/Control-Algorithm（ctrl_cpp/adrc）、PX4-Autopilot（stable 分支）
> 目标平台：PX4 多旋翼固件（SITL → HITL → 真机）

---

## 1. 文档目的与范围

本文档是"路线 B"的阶段二产物：给出**用 ADRC 替换 PX4 多旋翼角速率内环（mc_rate_control 控制律）**的模块级设计，作为阶段三（实现）的依据。

范围：

- 只替换角速率环控制律，**不修改**姿态环、位置/速度环、控制分配、模式管理与 failsafe；
- 保留原 PID 实现，通过参数做 A/B 切换，可随时回退；
- 覆盖接口设计、算法设计、参数清单、状态管理、调试手段、构建集成、验证计划。

不在范围内：算法理论研究、真机调参执行、量产固件发布流程。

## 2. 背景与动机

### 2.1 为什么选 ADRC 替换内环

| 维度 | PX4 原生角速率 PID | ADRC |
|---|---|---|
| 抗扰方式 | 积分项慢速消除稳态误差 | ESO 实时估计并主动补偿总扰动 |
| 微分信号 | 对误差求导（需滤波） | ESO 估计速度，天然平滑 |
| 对模型依赖 | 无 | 需标定 b0 |
| 鲁棒性 | 对参数摄动一般 | 对模型误差/扰动更鲁棒 |
| 调参复杂度 | 低 | 高（需带宽法初始化 + 标定流程） |

选 ADRC 的收益场景：行业机载（重载、大风、挂载变化、电机退化）下抗扰和鲁棒性优势明显。

### 2.2 设计原则

1. 最小侵入：新增独立模块，不删改原 mc_rate_control 源码；
2. 安全逻辑复用：解锁/着地/无数据/超时等判据与输出保护全部照抄原模块；
3. 可回退：A/B 切换参数，默认值为原 PID；
4. 可观测：新增调试话题，SITL 内可实时观察 ESO 内部状态；
5. 仿真先行：SITL 闭环对比达标前，不允许上硬件。

## 3. 总体架构

### 3.1 模块位置与文件布局

```
PX4-Autopilot/
├── msg/versioned/
│   └── AdrcStatus.msg                  # 新增调试话题（v1.17 消息定义位于 msg/versioned/）
├── ROMFS/px4fmu_common/init.d/rcS      # 按参数启动 A/B 模块（改动最小）
└── src/modules/
    ├── mc_rate_control/                # 原模块（不动）
    └── adrc_rate_control/              # 新模块
        ├── CMakeLists.txt
        ├── AdrcRateControl.hpp/.cpp    # ModuleBase + ScheduledWorkItem 主模块
        ├── Adrc.hpp/.cpp               # 移植自仓库 ctrl_cpp/adrc（纯算法，无 PX4 依赖）
        ├── adrc_params.c               # 参数定义（PX4_PARAM_DEFINE_*）
        └── test/
            └── test_adrc.cpp           # 单元测试（可选，阶段五）
```

### 3.2 数据流

```
mc_att_control ── vehicle_rates_setpoint ──┐
EKF2 ── vehicle_angular_velocity ──────────┤
EKF2 ── vehicle_angular_acceleration ──────┼──> adrc_rate_control ──> vehicle_torque_setpoint ──> 控制分配
land_detector ── vehicle_land_detected ────┤         │
commander ── vehicle_control_mode ─────────┘         └──> adrc_status（调试）
```

### 3.3 A/B 切换设计（关键决策）

避免两个模块同时向 `vehicle_torque_setpoint` 发布（最后写入者赢，存在竞态风险）。采用**"启动脚本按参数选择模块"**方案：

- 参数 `RATE_CTRL_MODE`：`0` = 原 mc_rate_control，`1` = adrc_rate_control；
- rcS 启动脚本读取该参数，只启动其中一个模块；
- 运行中切换：先 `stop` 当前模块再 `start` 目标模块（文档明确此限制，不做热切换）；
- 真机默认值锁定为 `0`（原 PID），验证通过后再由人工改默认值。

## 4. 接口设计

### 4.1 订阅话题

| 话题 | 字段（使用部分） | 用途 |
|---|---|---|
| `vehicle_rates_setpoint` | `roll`/`pitch`/`yaw` (rad/s) | 角速率期望（姿态环下发；手动/Acro 模式下由本模块自己生成并回发） |
| `vehicle_angular_velocity` | `xyz` (rad/s)、`timestamp` | 滤波后角速率实测值 + dt 基准；v1.17 通过 `SubscriptionCallbackWorkItem` **数据驱动**触发 |
| `vehicle_angular_acceleration` | `xyz` (rad/s²) | 备用（D 项/校验，默认不使用） |
| `vehicle_land_detected` | `landed`/`maybe_landed`/`freefall` | 输出抑制与状态管理 |
| `vehicle_control_mode` | `flag_armed`/`flag_control_rates_enabled` | 解锁沿检测、控制权判断 |
| `vehicle_status` | `arming_state` 等 | v1.17 新增订阅，用于状态判断（对齐原模块） |
| `parameter_update` | — | 参数热加载 |

### 4.2 发布话题

| 话题 | 字段 | 说明 |
|---|---|---|
| `vehicle_torque_setpoint` | `xyz`（归一化 -1~1） | 控制分配输入 |
| `vehicle_thrust_setpoint` | `xyz`（透传） | 与 torque 成对发布，值取自 `vehicle_rates_setpoint.thrust` 或保持原模块语义 |
| `vehicle_rates_setpoint` | 回发 | 手动/Acro 模式由本模块生成角速率期望并回发（**必须保留原逻辑**） |
| `rate_ctrl_status` | 状态 | 复用原模块的调试发布（或另发 `AdrcStatus`，见 7.1） |
| `AdrcStatus`（新增） | 见 7.1 | 调试用，仅仿真/测试阶段发布，真机可关 |

### 4.3 调度与 dt

- 调度方式与频率：**对齐原 mc_rate_control（v1.17）**——以 `vehicle_angular_velocity` 更新触发（`SubscriptionCallbackWorkItem` 数据驱动，随 IMU 250~1000 Hz），非固定周期轮询；
- dt 来源：`vehicle_angular_velocity.timestamp` 差分，单位换算为秒；
- dt 保护：clamp 到 `[0.0005, 0.005]` s，首帧不计算、只记录基线；
- 输出同步：以最新可用 setpoint + 角速度的**时间对齐**为准，禁止跨周期混用。

### 4.4 单位与坐标系

- 角速度：rad/s，机体坐标系（与原模块一致）；
- 控制输出：归一化力矩 [-1, 1]；
- 各轴独立 ADRC 实例，不做轴间解耦（与 PX4 原生一致）。

## 5. 算法设计

### 5.1 算法结构（每轴一套）

以仓库 `ctrl_cpp/adrc` 为基准移植，结构如下（`h` 为实际步长）：

**TD（跟踪微分器）**

```
v1(k+1) = v1(k) + h·v2(k)
v2(k+1) = v2(k) + h·fhan(v1(k) − v(k), v2(k), r, h)
```

**ESO（扩张状态观测器）**

```
e = z1 − y
z1 += h·(z2 − β01·e)
z2 += h·(z3 − β02·fal(e, 0.5, δ) + b0·u)
z3 += h·(−β03·fal(e, 0.25, δ))
```

**NLSEF（非线性反馈律）**

```
e1 = v1 − z1
e2 = v2 − z2
u0 = β1·fal(e1, α1, δ) + β2·fal(e2, α2, δ)
u  = (u0 − z3) / b0
```

其中 `fal(e, α, δ)` 为非线性函数（线性区间 δ 内退化为 e/δ^(1−α)）。具体数值实现以仓库代码为准，移植时保留其浮点运算结构。

### 5.2 与仓库代码的差异点（移植必改）

| 项目 | 仓库原版 | PX4 移植版 |
|---|---|---|
| dt | 固定 `dt` | 动态时间戳差分 + clamp |
| 输出 | 无约束 | 每轴 clamp 到 [-1, 1]，超限时回馈抑制 ESO |
| 状态管理 | 无 | 解锁/着地/超时清零（见第 6 节） |
| 参数 | 代码内配置 | PX4 param 系统 |
| 数值类型 | float | float（保持），避免 double 的 MCU 开销 |
| 启动瞬态 | 无要求 | 解锁前沿必须将 z1 初始化为当前角速度、z2/z3 清零 |

### 5.3 各轴实例化

- roll / pitch / yaw 各一个 ADRC 实例，参数独立；
- **yaw 特殊处理**：偏航通道转动惯量小、带宽需求低，ESO 带宽与 NLSEF 增益取 roll/pitch 的 1/2~2/3 起步；b0 单独标定；
- 参数按 `_ROLL_` / `_PITCH_` / `_YAW_` 后缀区分。

### 5.4 b0 标定方法（设计内约定）

b0 是"控制量→角加速度"的增益：`ω̇ ≈ b0·u`（u 为归一化力矩）。

1. 粗估：由机架转动惯量 J（CAD 或摆荡法）得 `b0 ≈ 1/J`（归一化单位下需再乘力矩系数）；
2. SITL 辨识：固定油门悬停，对各轴施加已知方波力矩指令，记录角加速度响应，`b0 = Δω̇/Δu`；
3. 真机复核：悬停小幅激励，观察 ESO 的 z3 是否近似为 0（b0 正确时 z3 只含真实扰动，不应包含系统性偏差）。

### 5.5 ESO 参数初始化（带宽法）

为缓解"参数多"问题，β 按观测器带宽 ω₀ 初始化，再微调：

```
β01 = 3·ω₀
β02 = 3·ω₀²
β03 = ω₀³
```

- 起步建议：roll/pitch `ω₀ ≈ 40~80 rad/s`，yaw `ω₀ ≈ 20~40 rad/s`；
- 调参顺序固定：ESO（z1/z2/z3 跟上）→ TD（r）→ NLSEF（β1/β2、α1/α2）。

## 6. 状态管理与安全逻辑

### 6.1 ESO/控制器状态复位条件

| 事件 | 动作 |
|---|---|
| 解锁上升沿 | z1 = 当前角速度，z2 = 0，z3 = 0，v1/v2 = 0，u = 0 |
| 着地 / maybe_landed | 输出 0，冻结或清零 ESO（清零，防止解锁冲激） |
| 自由落体 | 输出 0，沿用原模块逻辑 |
| 无有效 setpoint / 角速度超时 | 输出 0，状态保持（等待恢复） |
| 参数更新 | 在线生效；ESO 状态不重置（β 变化影响有限） |

### 6.2 输出保护

- 每轴输出 clamp [-1, 1]；
- clamp 发生时，按抗饱和方式回馈修正 ESO 的 z3 更新（防止 ESO 状态持续积分发散）——参考仓库 PID 的 Kaw 思路实现一个简易版本；
- `vehicle_torque_setpoint` 的 `timestamp` 必须每次更新；
- 控制权不在本模块（`flag_control_rates_enabled == false`）时一律输出 0。

### 6.3 failsafe

- 不新增任何自主行为；超时/无数据时输出 0 并交回 commander 原有处理；
- 保留原模块的"禁止输出前向偏置"约束（解锁前 u 恒为 0）。

## 7. 调试与日志设计

### 7.1 新增 `msg/adrc_status.msg`

```
uint64 timestamp
float[3] z1     # ESO 状态估计：角速率
float[3] z2     # ESO 状态估计：角加速度
float[3] z3     # ESO 总扰动估计
float[3] v1     # TD 参考
float[3] v2     # TD 参考微分
float[3] u      # 输出（clamp 前）
float[3] e1     # NLSEF 误差 1
float[3] e2     # NLSEF 误差 2
```

### 7.2 观测手段

- SITL：`listener adrc_status`、`listener vehicle_torque_setpoint`；
- 真机：加入 logger 配置（`src/modules/logger/logged_topics.cpp` 增加该话题），调参时以 1 kHz 记录；
- 与 `vehicle_rates_setpoint`、`vehicle_angular_velocity` 同帧对比即可判断 ESO 是否收敛。

## 8. 参数清单（PX4 param）

命名规则：`ADRC_<AXIS>_<NAME>`（AXIS ∈ ROLL/PITCH/YAW），共享参数用 `ADRC_<NAME>`。

### 8.1 模式切换

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `RATE_CTRL_MODE` | INT32 | 0 | 0=原 PID，1=ADRC（改后需重启模块） |

### 8.2 ADRC 参数（每轴 12 个）

| 参数 | 类型 | 说明 |
|---|---|---|
| `ADRC_<AXIS>_R` | FLOAT | TD 快速跟踪因子 |
| `ADRC_<AXIS>_B0` | FLOAT | 控制增益（按 5.4 标定） |
| `ADRC_<AXIS>_DELTA` | FLOAT | fal 线性区间宽度 |
| `ADRC_<AXIS>_ESO_B01` | FLOAT | ESO β01 |
| `ADRC_<AXIS>_ESO_B02` | FLOAT | ESO β02 |
| `ADRC_<AXIS>_ESO_B03` | FLOAT | ESO β03 |
| `ADRC_<AXIS>_NL_B1` | FLOAT | NLSEF β1 |
| `ADRC_<AXIS>_NL_B2` | FLOAT | NLSEF β2 |
| `ADRC_<AXIS>_NL_A1` | FLOAT | NLSEF α1（0<α1<1） |
| `ADRC_<AXIS>_NL_A2` | FLOAT | NLSEF α2（α2>1） |
| `ADRC_<AXIS>_U_MAX` | FLOAT | 输出上限（默认 1） |
| `ADRC_<AXIS>_U_MIN` | FLOAT | 输出下限（默认 -1） |

实现方式：`adrc_params.c` 用 `PX4_PARAM_DEFINE_FLOAT` 定义；模块内 `param_find` + `param_get`，参数更新话题触发后刷新缓存（避免每周期查询）。

## 9. 构建与集成

### 9.1 `CMakeLists.txt`

```cmake
px4_add_module(
    MODULE modules__adrc_rate_control
    MAIN adrc_rate_control
    STACK_MAIN 2500
    SRCS
        AdrcRateControl.cpp
        Adrc.cpp
    DEPENDS
)
```

### 9.2 启动集成

- rcS 中按 `RATE_CTRL_MODE` 决定启动 `mc_rate_control` 还是 `adrc_rate_control`；
- 两模块在 CMake 构建中同时编译，保证切换无需重新刷固件；
- SITL 手动测试命令：`module start adrc_rate_control` / `module stop mc_rate_control`。

### 9.3 v1.17 参考模板

- 模块骨架参考：`src/templates/template_module/`；
- 消息定义位置：`msg/versioned/`（新增 `AdrcStatus.msg` 放这里，文件名用驼峰）；
- 数据驱动调度参考：`src/modules/mc_rate_control/MulticopterRateControl.cpp` 中 `_vehicle_angular_velocity_sub` 的 `SubscriptionCallbackWorkItem` 用法；
- 手动/Acro 模式回发 `vehicle_rates_setpoint` 的逻辑必须从原模块移植，不能丢弃。

### 9.4 构建与仿真命令

```bash
# SITL（先验证）
make px4_sitl gazebo-classic

# 目标硬件（阶段五）
make px4_fmu-v6x_default    # 以实际飞控板为准
```

## 10. 验证计划

### 10.1 测试用例（SITL）

| 用例 | 输入 | 通过标准 |
|---|---|---|
| 开环冒烟 | 模块启动、无指令 | 正常调度，无报错，话题有数据 |
| 阶跃响应 | 各轴 30°/s 阶跃 | 无发散；超调 < 原 PID 或相当；稳态误差 ≈ 0 |
| 正弦跟踪 | 各轴 1~3 Hz 正弦 | 幅值/相位滞后不劣于原 PID |
| 抗扰 | 悬停中注入持续/突变力矩扰动 | ESO z3 快速跟踪扰动；姿态偏差恢复快于原 PID |
| 饱和 | 大幅指令触发 u 限幅 | 无积分/ESO 发散，退出饱和后正常 |
| 模式切换 | 起飞→悬停→降落，中途切模式 | 全程可回退，无解锁冲激 |
| 超时/丢数据 | 人为停发 setpoint | 输出 0，failsafe 行为与原模块一致 |

### 10.2 A/B 对比指标

- 上升时间、超调量、稳态误差；
- 抗扰恢复时间（扰动注入到偏差回到 ±5%）；
- 输出峰值与抖动量（不劣于原 PID）；
- 状态切换瞬态（解锁、起降）无尖峰。

### 10.3 验证顺序

1. 单元测试（Adrc 类，PC 上跑，含边界值）；
2. SITL 全用例；
3. HITL（同一套固件）；
4. 真机：悬停 → 小幅机动 → 大风/负载测试；全程手动模式可接管，RATE_CTRL_MODE 可切回 0。

## 11. 风险与缓解

| 风险 | 影响 | 缓解 |
|---|---|---|
| b0 标定不准 | 发散或性能差 | 5.4 三步标定；仿真先行；参数可在线调 |
| ESO 参数耦合难调 | 调试周期长 | 5.5 带宽法初始化；adrc_status 可视化 |
| 解锁冲激 | 炸机风险 | 6.1 解锁沿复位；SITL 重点验证 |
| 双模块竞态发布 | 不可预期行为 | 3.3 启动脚本互斥方案 |
| 许可证问题 | 商用/分发法律风险 | 移植前取得仓库作者书面授权（仓库无 LICENSE） |
| PX4 版本漂移 | 接口不兼容 | 锁定 stable 分支；文档按该版本归档 |

## 12. 实施里程碑

| 里程碑 | 内容 | 完成标志 |
|---|---|---|
| M1 | 空模块透传 | SITL 可启动，topic 链路通 |
| M2 | Adrc 算法接入 + 参数系统 | 参数可调，输出有响应 |
| M3 | 安全逻辑 + 状态管理 | 解锁/着地/超时行为正确 |
| M4 | A/B 切换 | rcS 按参数互斥启动 |
| M5 | SITL 调参与对比 | 10.1 全用例通过 |
| M6 | HITL | 与 SITL 一致 |
| M7 | 真机验证 | 达标后默认值可切为 1 |

---

## 13. 实施状态（2026-08-10）

已完成 M1–M4 的代码实现并通过 `make px4_sitl_default` 编译链接：

- 新增模块 `src/modules/adrc_rate_control/`（Adrc 算法类 + AdrcRateControl 模块 + 参数 + Kconfig + CMake）；
- 新增消息 `msg/AdrcStatus.msg`（已注册进 `msg/CMakeLists.txt`）；
- 板级启用：`boards/px4/sitl/default.px4board` 增加 `CONFIG_MODULES_ADRC_RATE_CONTROL=y`；
- A/B 切换：`rc.mc_apps` / `rc.vtol_apps` 按 `RATE_CTRL_MODE` 互斥启动；
- 36 个 `ADRC_*` 参数 + `RATE_CTRL_MODE` 全部注册，`adrc_status` 话题已生成；
- 与上游 `ctrl_cpp/adrc` 的差异：改用标准补偿公式 `u = (u0 − z3)/b0`（上游 `u0 − z3/b0` 与 ESO 公式不自洽），输出限幅后的实际控制量回馈给 ESO 实现自然抗饱和。

注意：默认 `RATE_CTRL_MODE=0`（原 PID），切换前必须在仿真中完成 b0 标定与调参。

---

## 附：与仓库代码的对接点

- 直接移植：`ctrl_cpp/adrc.cpp`、`ctrl_cpp/adrc.h`（纯 C++17，无平台依赖）；
- 参考不移植：`ctrl_cpp/utils.*`（与算法无关）、Python 版 `controller/siso/adrc.py`（仅对照公式）；
- 移植前先完成作者授权确认（无 LICENSE）。
