# 从零理解：ADRC 替换 PX4 角速率环（学习笔记）

> 配套代码：`/home/liuyao/my_px4_project/PX4-Autopilot`（PX4 v1.15.4）
> 本文回答三个问题：怎么替换的？改了哪些参数？参数是怎么"增加"的？

---

## 1. 替换点：PX4 控制链的哪一环被换了

PX4 多旋翼控制是串级结构：

```
位置环 mc_pos_control -> 姿态环 mc_att_control -> 角速率环 mc_rate_control(PID) -> 控制分配 -> 电机
```

我们替换的是 **mc_rate_control**：它的输入是"期望角速率"（`vehicle_rates_setpoint`，姿态环生成）和"实测角速率"（`vehicle_angular_velocity`，陀螺仪滤波后），输出是"力矩/推力"（`vehicle_torque_setpoint` / `vehicle_thrust_setpoint`）。

**替换的关键思想**：新模块 `adrc_rate_control` 和原模块的 uORB 接口**完全一致**（订阅同样的话题、发布同样的话题），所以姿态环、控制分配、模式管理全部不用动，只有这一环的控制律变了。

## 2. "怎么替换"的四个机制层面

### 2.1 新模块源码

```
src/modules/adrc_rate_control/
|-- Adrc.hpp / Adrc.cpp          # 纯算法：LADRC（线性ESO+ωc）与经典 fal ADRC
|-- AdrcRateControl.hpp/.cpp     # PX4 模块外壳：订阅/发布/调度/安全逻辑
|-- adrc_params.c                # 参数定义
|-- CMakeLists.txt               # 构建接入（px4_add_module）
`-- Kconfig                      # 模块开关
```

`AdrcRateControl` 类继承 `ModuleBase + ModuleParams + px4::WorkItem`（和原模块同款），`Run()` 由陀螺仪数据驱动（`SubscriptionCallbackWorkItem`），每个周期：

1. 时间戳差分得到 dt（并做 0.125ms~20ms 的保护）；
2. 读角速率期望（或 ACRO 模式用摇杆生成）；
3. 每轴调用 `Adrc::update()` 算出力矩；
4. 发布力矩/推力话题；同时发布 `adrc_status`（调试）。

安全逻辑照搬原模块：解锁沿完整复位 ESO、着地时只清 z3（等价原 PID 清积分）、输出 clamp、非有限值保护、电池缩放。

### 2.2 构建接入（让固件包含这个模块）

1. `Kconfig` 声明开关 `MODULES_ADRC_RATE_CONTROL`；
2. 板级配置 `boards/px4/sitl/default.px4board` 加 `CONFIG_MODULES_ADRC_RATE_CONTROL=y`；
3. 顶层 CMake 会 `foreach(module ${config_module_list}) add_subdirectory(src/${module})`，自动编译模块目录里的 `px4_add_module(...)`；
4. 同时把 `mc_rate_control` 和 `mc_autotune_attitude_control` 从板配置里去掉（不再编译）。

### 2.3 启动接入（开机运行哪个模块）

多旋翼启动脚本 `ROMFS/px4fmu_common/init.d/rc.mc_apps` 改为无条件启动：

```sh
adrc_rate_control start
```

**踩过的坑**：上游 `ROMFS/.../init.d/CMakeLists.txt` 把 `rc.mc_apps` 的打包绑定在 `CONFIG_MODULES_MC_RATE_CONTROL` 上——关掉 PID 模块后整个多旋翼启动脚本都进不了固件，导致 ADRC 根本不会被启动。修复为：

```cmake
if(CONFIG_MODULES_MC_RATE_CONTROL OR CONFIG_MODULES_ADRC_RATE_CONTROL)
```

### 2.4 参数迁移（避免固件缺参）

`MC_ACRO_*` 和 `MC_BAT_SCALE_EN` 原本定义在 mc_rate_control 里，模块被禁用后这些参数会从固件消失（姿态环/自动调参可能引用）。把它们**原样搬进**我们的 `adrc_params.c`。

## 3. 重点：PX4 参数是怎么"增加"的（完整流水线）

### 第 1 步：写定义

`adrc_params.c` 里用宏声明参数，注释里的 `@min/@max/@group` 是元数据（QGC 显示、校验用）：

```c
/**
 * Roll LADRC controller bandwidth w_c [rad/s]
 *
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 0.5
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_CW, 15.0f);
```

### 第 2 步：构建系统"自动发现"

`src/lib/parameters/CMakeLists.txt` 里有：

```cmake
file(GLOB_RECURSE param_src_files ${PX4_SOURCE_DIR}/src/*params.c ...)
```

**只要文件名匹配 `*params.c` 且放在 src/ 下，就会被扫描**——这就是为什么我们不用改任何 CMake 就能注册参数。

### 第 3 步：代码生成（构建时自动）

`px_process_params.py` / `generate_params.py` 解析 `PARAM_DEFINE_*`，生成：

- `build/.../parameters.json`（QGC 和文档用的元数据）；
- `build/.../src/lib/parameters/px4_parameters.hpp`（含 `px4::params::ADRC_ROLL_CW` 枚举和元数据表）。

### 第 4 步：模块里使用

模块头文件用 `DEFINE_PARAMETERS` 声明"参数句柄"：

```cpp
DEFINE_PARAMETERS(
    (ParamFloat<px4::params::ADRC_ROLL_CW>) _param_adrc_roll_cw,
    ...
)
```

代码里 `_param_adrc_roll_cw.get()` 取值；参数变化时 `parameter_update` 话题通知模块，`updateParams()` + `parameters_updated()` 应用新值（LADRC 里重新计算 β01=2·ESO_W、β02=ESO_W²）。

### 第 5 步：运行时生效

- QGC / pxh 里 `param set ADRC_ROLL_CW 12`；
- 固件广播 `parameter_update`；
- 模块热更新（无需重启）。

**命名约束**：参数名不超过 16 字符。我们踩过：`ADRC_PITCH_CTRL_W`（17 字符）编译被拒，改名为 `ADRC_PITCH_CW`。

## 4. 我们实际改动了哪些参数

### 4.1 新增参数（adrc_params.c）

| 参数 | 含义 | 定版值 |
|---|---|---|
| `ADRC_ESO_MODE` | 0=经典 fal ADRC，1=LADRC | 1 |
| `ADRC_CTRL_LAW` | 0=非线性律，2=线性律（模式0用） | 2 |
| `ADRC_<轴>_B0` | 植物增益（必须标定） | 100/100/20 |
| `ADRC_<轴>_CW` | 控制带宽 ωc | 11/11/8 |
| `ADRC_<轴>_ESO_W` | 观测带宽 ωo | 50/50/45 |
| `ADRC_<轴>_R/_DELTA/_B01/_B02/_B03/_NB1/_NB2/_A1/_A2/_UMX/_UMN` | 经典 fal 模式参数（模式0用） | 默认值 |

### 4.2 从 mc_rate_control 迁移的参数

`MC_ACRO_R_MAX/P_MAX/Y_MAX`、`MC_ACRO_EXPO/EXPO_Y/SUPEXPO/SUPEXPOY`、`MC_BAT_SCALE_EN`。

### 4.3 删除的参数

`RATE_CTRL_MODE`（早期 A/B 切换开关，去掉 PID 后不再需要）。

### 4.4 原 PID 参数（MC_ROLLRATE_P 等）

随 mc_rate_control 一起从固件移除（自动调参模块也一并禁用）。

## 5. 调试消息与日志是怎么加的

### 5.1 新 uORB 消息

1. 写 `msg/AdrcStatus.msg`（z1/z2/z3/v1/v2/u/e1/e2，每轴 3 个）；
2. 在 `msg/CMakeLists.txt` 的 `set(msg_files ...)` 列表里加一行；
3. 构建时自动生成话题 `adrc_status`、头文件 `uORB/topics/adrc_status.h`；
4. 模块里 `uORB::Publication<adrc_status_s>` 发布。

### 5.2 写入飞行日志

`src/modules/logger/logged_topics.cpp`：

```cpp
add_optional_topic("adrc_status", 5);
```

**注意**：这里的第二个参数是**采样间隔毫秒**，不是速率：写 200 = 200ms = 5Hz（我们一开始写错了，日志只有 5Hz）；写 5 = 5ms = 200Hz。

## 6. 调参回顾（数据说话）

| 轮次 | 主要改动 | 结果 |
|---|---|---|
| 初版 | fal ADRC，b0=1 | 起飞即振荡（b0 错两个数量级） |
| 线性律 | ADRC_CTRL_LAW=2，b0=100/100/50 | 能悬停，打杆 3Hz 振荡 |
| LADRC | ADRC_ESO_MODE=1 | 稳定飞行，z3 仍偏大 |
| b0 修正 | yaw b0 50->20 | yaw z3 从 ±43 降到 ±1.5 |
| 带宽 | CW 11/11/8，ESO_W 50/50/45 | 悬停抖动 15 mrad/s，跟踪增益约 1 |

## 7. 自测问题

1. 为什么新模块接口和原模块一致就能"无缝替换"？（答：uORB 解耦，上下游只看话题）
2. 加一个参数需要改哪些文件？哪一步是"自动"的？（答：定义和使用是手写；glob 发现与代码生成自动）
3. `add_optional_topic("x", 200)` 是 200Hz 还是 5Hz？（答：5Hz，参数是间隔毫秒）
4. 禁用 mc_rate_control 后为什么会缺 MC_ACRO_* 参数？（答：参数定义随模块消失，需迁移）
5. 为什么 rc.mc_apps 会"消失"？（答：ROMFS 打包按 CONFIG 条件，需加 ADRC 条件）
6. b0 为什么必须真机重标？（答：取决于惯量 J 和最大力矩，SITL 模型值不能直接搬）
