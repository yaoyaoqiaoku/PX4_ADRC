# Gazebo 加风 / 环境扰动：ADRC 抗扰测试指南

> 日期：2026-08-13
> 你的环境：gz-sim 8.14（新 Gazebo）+ PX4 1.15 + x500 模型（log_12 即此环境，SIM_GZ_EN=1）

---

## 0. 先说两个关键事实

1. **gz-sim 8 的 world 里可以定义风，但 PX4 的无人机模型默认没开 `enable_wind`——风吹不动它。** 要加风必须先改模型文件。
2. **gz-sim 8 的风是恒定风场，没有阵风模型。** 阵风/瞬态扰动要么用外力（wrench）注入，要么换 Gazebo Classic（有完整的阵风插件）。

三条路都可以用，按需求选：

| 需求 | 用哪个 |
|---|---|
| 持续侧风，看稳态抗扰 | A：gz-sim 恒定风（改两个文件） |
| 阵风 / 挂载突变 / 任意扰动序列 | B：gz-sim wrench 外力注入（推荐，最灵活） |
| 真实风谱：持续风 + 阵风 + 渐变风 | C：Gazebo Classic（需要安装 gazebo11） |

---

## 1. 方法 A：gz-sim 8 恒定风（最简单）

### 1.1 给模型开风

编辑 `PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf`，在 `<link name="base_link">` 内加一行：

```xml
<link name="base_link">
  <enable_wind>true</enable_wind>
  <!-- 原有内容不动 -->
</link>
```

只给 base_link 开就够，机臂/电机不需要。

### 1.2 设风速

编辑 world 文件（直接用现成的 `windy.sdf`，或复制 `default.sdf` 改）：

```xml
<wind>
  <linear_velocity>5 0 0</linear_velocity>   <!-- ENU，m/s；x=东向侧风 -->
</wind>
```

默认的 `windy.sdf` 是 `10 10 10`（三轴都有风，太大，建议先改小）。

### 1.3 启动

```bash
# 终端 1：PX4（你原来的启动方式，比如 make px4_sitl_default）
# 终端 2：Gazebo 指定 windy world
python Tools/simulation/gz/simulation-gazebo --world windy
```

### 1.4 验证风生效

起飞悬停后看几个 topic：

```bash
listener vehicle_local_position   # x/y 会被吹着漂移
listener vehicle_attitude         # roll/pitch 出现对抗风的倾斜角
listener adrc_status              # z3 出现稳态估计值（ESO 把风当扰动）
```

如果完全没反应，先确认 `enable_wind` 加对了、world 里 `<wind>` 存在。gz-sim 8 的风力模型简单，**从小风速（2 m/s）开始试**，别一上来就 10。

---

## 2. 方法 B：外力 / 阵风注入（推荐给 ADRC 测试）

所有 PX4 gz world 都已加载 `ApplyLinkWrench` 插件，直接用 `gz topic` 给机体施加力/力矩，不用改任何文件。

### 2.1 基本命令

```bash
# 给 x500 的 base_link 施加 -8 N 的 x 向力（模拟侧风阵风）
gz topic -t /world/windy/wrench -m gz.msgs.EntityWrench \
  -p "entity: {name: 'x500::base_link'} wrench: {force: {x: -8, y: 0, z: 0}}"

# 撤掉（发零力）
gz topic -t /world/windy/wrench -m gz.msgs.EntityWrench \
  -p "entity: {name: 'x500::base_link'} wrench: {force: {x: 0, y: 0, z: 0}}"
```

world 名要和启动的一致（`windy` / `default` / `baylands`）。如果 `entity.name` 不对，用 `gz model --info` 查实际名称。

### 2.2 做成扰动序列（用现成工具 `gust_inject.py`）

这是测试 ADRC 动态抗扰的最佳方式：

| 扰动类型 | 力序列 | 测什么 |
|---|---|---|
| 阵风阶跃 | 0 → -8N 保持 3s → 0 | z3 响应速度、角速率峰值、恢复时间 |
| 阵风脉冲 | 0 → -8N 0.3s → 0 | ESO 带宽（ESO_W）与振荡倾向 |
| 正弦扫风 | F = A·sin(2πft)，f 从 0.5→5Hz 扫 | 找 ADRC 补偿回路的谐振点（对应极限环） |
| 挂载突变 | z 向力突然 +/− | KI / b0 失配补偿 |

配套工具：[gust_inject.py](/home/liuyao/文档/Codex/2026-08-10/new-chat/outputs/gust_inject.py)，一行命令即可，自动发零力收尾：

```bash
# 先自检（确认 world 名 / link 名）
python3 gust_inject.py --check

# 阵风阶跃：-8N 侧风保持 3s
python3 gust_inject.py --world windy --force 8,0,0 --wave step --duration 3

# 阵风脉冲：0.5s
python3 gust_inject.py --world windy --force 8,0,0 --wave pulse --duration 0.5

# 正弦扫风 0.5->5Hz（找谐振点）
python3 gust_inject.py --world windy --force 6,0,0 --wave sine --freq 0.5 --freq_end 5 --duration 40

# 挂载突变：z 向 -5N 保持 10s
python3 gust_inject.py --world windy --force 0,0,-5 --wave step --duration 10

# 重复阵风：3s 阶跃，每 6s 一次，共 5 次
python3 gust_inject.py --world windy --force 8,0,0 --wave step --duration 3 --interval 6 --reps 5
```

脚本会在每次注入时打印相对时间戳，方便和 ulog 时间轴对齐。

注意：wrench 力和风会叠加，别同时开两个相同方向的扰动。

---

## 3. 方法 C：Gazebo Classic（完整风模型，推荐装）

gz-sim 8 的风太简单，PX4 对 **Gazebo Classic** 有专门的风插件，能生成持续风 + 阵风 + 渐变风，最适合验证"ADRC 抗扰是否真的比 PID 好"。

### 3.1 安装

```bash
sudo apt install gazebo11 libgazebo11-dev
```

### 3.2 启动带风的 world

```bash
make px4_sitl gazebo-classic_windy          # 持续风 + 阵风
make px4_sitl gazebo-classic_ramped_up_wind  # 渐变风
```

### 3.3 风参数（`Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/windy.world`）

| 参数 | 含义 | windy 默认 |
|---|---|---|
| `windVelocityMean` | 基础风速 m/s | 4 |
| `windVelocityMax` | 风速上限 | 20 |
| `windVelocityVariance` | 风速随机波动 | 0 |
| `windDirectionMean` | 风向（ENU 向量） | 0 1 0 |
| `windDirectionVariance` | 风向随机波动 | 0 |
| `windGustStart` | 阵风开始时间 s | 0 |
| `windGustDuration` | 阵风持续 s | 0 |
| `windGustVelocityMean/Max/Variance` | 阵风强度 | 0/20/0 |
| `windGustDirectionMean` | 阵风方向 | 1 0 0 |

`ramped_up_wind.world` 额外有：

| 参数 | 含义 | 默认 |
|---|---|---|
| `windRampStart` | 渐变开始时间 s | 60 |
| `windRampWindVectorComponents` | 渐变目标风速向量 | 0 15 0 |
| `windChangeRampDuration` | 渐变持续时间 s | 30 |

例：把阵风改成"第 30s 起、3 m/s 的侧向阵风 5 秒"：

```xml
<windGustStart>30</windGustStart>
<windGustDuration>5</windGustDuration>
<windGustVelocityMean>3</windGustVelocityMean>
<windGustVelocityMax>5</windGustVelocityMax>
<windGustDirectionMean>1 0 0</windGustDirectionMean>
```

---

## 4. 用这些环境测 ADRC 的什么（测试矩阵）

| 场景 | 施加方式 | ADRC 关注指标 | 对比对象 |
|---|---|---|---|
| 稳态侧风 | A 恒定风 / C 持续风 / B 恒定 wrench | 悬停位置漂移、姿态倾斜角、z3 稳态值 | PID vs ADRC；gamma=1 vs 0.7 |
| 阵风阶跃 | B 阶跃 wrench / C windGust | z3 上升时间、角速率峰值、恢复时间、输出能量 | ESO_W 10/15/20；TAU 开/关 |
| 阵风脉冲 | B 脉冲 wrench | 是否有 2~4Hz 振铃（对应极限环） | FLT/TAU/Z3MAX 开/关 |
| 正弦扫风 | B 正弦 wrench 脚本 | 补偿回路谐振频率、相位裕度表现 | 对应控制器级仿真 A 组 |
| 挂载突变 | B z 向力突变 | 高度偏差、KI 收敛 | KI 开/关 |

**记录 topic**（ulog 都会记）：`vehicle_local_position`、`vehicle_attitude`、`vehicle_angular_velocity`、`adrc_status`（z3/z3raw/sp）、`vehicle_torque_setpoint`。

**看结果**：同一扰动序列跑 PID 和 ADRC 各一组，对比角速率 RMS、姿态偏差积分、z3 触限次数（z3raw != z3）、控制能量。

---

## 5. 注意事项

1. **gz-sim 8 的风力模型粗糙**：先 2 m/s 验证再加大，别直接信数值。
2. **enable_wind 只影响开了它的 link**：base_link 开即可，其他不动。
3. **wrench 注入最可控**：数值、方向、时序都由你定，且不需要动模型/world 文件——建议作为主力方法。
4. **Classic 的 wind_plugin 对 PX4 模型有效**（iris 等官方模型默认响应 world_wind）。
5. 想测"阵风+延时+噪声"同时存在的极限环，优先用控制器级仿真（见《ADRC参数仿真测试指南》）快速扫参数，再用本指南的 Gazebo 环境做整机确认。
