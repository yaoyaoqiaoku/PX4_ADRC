# ADRC 真机调试测试指南（v4 固件）

> 配套固件：PX4-Autopilot v1.15.4 + adrc_rate_control v4（三阶增广 ESO + 分配器饱和反馈）
> 机架：66cm 对角轴距 / 3.5kg / U3515 电机 / 14 寸桨 / 6S / PX4_FMU_V6C
> 日期：2026-08-14
> 配套文件：`params_adrc_real.txt`（参数预设）、`calibrate_b0.py`（b0 标定）、`ADRC真机优化-底层改进方案.md`（理论背景）

---

## 1. 前置条件（飞前必须完成）

### 1.1 两份固件

**必须手边有 PID 固件**——这是唯一的安全回退手段。当前 ADRC 固件已禁用 `mc_rate_control`，无法在运行时切回 PID。

```
# 在 PX4-Autopilot 目录下编译两份固件：
# ADRC 固件（已编译好，直接用）
make px4_fmu-v6c_default          # → build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4

# PID 固件（需要临时改两个文件）
# 1. boards/px4/fmu-v6c/default.px4board：
#    CONFIG_MODULES_ADRC_RATE_CONTROL=y → 删除
#    CONFIG_MODULES_MC_RATE_CONTROL=y → 加上
# 2. ROMFS/px4fmu_common/init.d/rc.mc_apps：
#    adrc_rate_control start → mc_rate_control start
make px4_fmu-v6c_default          # → 另存为 px4_fmu-v6c_pid_default.px4
```

**两份 .px4 文件都拷到 SD 卡/USB 随身带**——真机出问题时 QGC 一键刷回。

### 1.2 硬件检查（每次飞行前）

- [ ] 电池满电（6S ≥ 25.0V 开机，≥ 23.5V 悬停中）——日志显示 20.4V 时推力严重不足
- [ ] 四支桨动平衡 / 无裂纹 / 拧紧
- [ ] 电机座、机臂螺丝、机身上盖紧固
- [ ] IMU 减震（硅胶垫/泡沫）到位
- [ ] 飞控安装牢固、无晃动
- [ ] GPS fix、罗盘正常
- [ ] 遥控器解锁开关/紧急降落按钮可用

### 1.3 参数验证（烧录后必做）

烧录 ADRC 固件后，在 QGC 参数界面或 MAVLink Console 确认：

```
param show ADRC_ESO_MODE       # 应为 1（LADRC）
param show ADRC_ROLL_CW        # 初始应为 8
param show ADRC_ROLL_ESO_W     # 初始应为 20
param show ADRC_ROLL_B0        # 初始应为 90
param show ADRC_ROLL_TAU       # 初始应为 0.02
param show ADRC_ROLL_GAMMA     # 初始应为 0.7
param show ADRC_ROLL_FLT       # 初始应为 30
param show MC_AIRMODE          # 应为 1
param show MC_ROLL_P           # 应为 4.5
```

---

## 2. 安全守则

1. **每轮只改一个参数组**（同一轴的同一类参数算一组），改完 `param save`，飞一次验证后再动下一个；
2. **首次飞行只做低空悬停**（0.5~1m），不打杆，持续 30 秒即可；
3. **手不离解锁开关**——出现 2~4Hz 快速抖动或 1~2Hz 大幅摇摆立即降落/切手动；
4. **每次试飞只改一组参数**，改后在地面先小油门（30~40%）观察机身有无剧烈摆动，无异常再正常起飞；
5. **ADRC 出问题 → 刷 PID 固件**——不要在真机上"硬调"，你已经验证过这条路走不通。

---

## 3. PID 基线飞行（必做，不做不飞 ADRC）

**目的**：拿到"及格线"数据，后续所有 ADRC 日志都和它对比。

1. 刷 PID 固件，起飞悬停 30 秒，记录 .ulg 日志；
2. 落地后用以下命令提取角速率 RMS 和 FFT（或发给我分析）：

```python
# 快速检查（需 pyulog + numpy）
from pyulog import ULog
import numpy as np
u = ULog('log_pid.ulg')
d = u.get_dataset('vehicle_angular_velocity')
t = (d.data['timestamp'] - d.data['timestamp'][0]) * 1e-6
for ax, nm in [(0,'roll'),(1,'pitch'),(2,'yaw')]:
    x = d.data['xyz[%d]'%ax]
    print('%s: RMS %.2f deg/s, |max| %.1f deg/s' % (nm, np.degrees(np.std(x)), np.degrees(np.max(np.abs(x)))))
```

3. **记录 PID 基线值**（示例，你的实际值以日志为准）：

| 轴 | 悬停 RMS (°/s) | 主峰频率 (Hz) | 备注 |
|---|---|---|---|
| roll | ~7-13 | ~1.0 | 正常悬停/微风 |
| pitch | ~7-12 | ~1.0 | |
| yaw | ~3-8 | ~0.5 | |

**PID 基线就是 ADRC 的及格线**——ADRC 的悬停 RMS 不应超过 PID 的 1.5 倍。

---

## 4. ADRC 调参流程（逐步，每步飞一次）

### 第 0 步：设置起点参数

烧录 ADRC 固件后，在 QGC MAVLink Console 逐行粘贴：

```bash
# ---- 控制增益（cw/b0 ≈ 0.089，约为 PID 的 60%，保守起步）----
param set ADRC_ROLL_CW 8
param set ADRC_PITCH_CW 8
param set ADRC_YAW_CW 5
param set ADRC_ROLL_ESO_W 20
param set ADRC_PITCH_ESO_W 20
param set ADRC_YAW_ESO_W 12
param set ADRC_ROLL_B0 90
param set ADRC_PITCH_B0 90
param set ADRC_YAW_B0 8

# ---- 硬化项（防 2~3Hz 极限环，全程保持）----
param set ADRC_ROLL_TAU 0.02
param set ADRC_PITCH_TAU 0.02
param set ADRC_YAW_TAU 0.01
param set ADRC_ROLL_GAMMA 0.7
param set ADRC_PITCH_GAMMA 0.7
param set ADRC_YAW_GAMMA 0.6
param set ADRC_ROLL_FLT 30
param set ADRC_PITCH_FLT 30
param set ADRC_YAW_FLT 20

# ---- 姿态环降增益（减少对内环带宽的需求）----
param set MC_ROLL_P 4.5
param set MC_PITCH_P 4.5
param set MC_YAW_P 4.0
param set MC_AIRMODE 1

# ---- 关闭硬化旋钮（暂不需要）----
param set ADRC_ROLL_FF 0
param set ADRC_PITCH_FF 0
param set ADRC_ROLL_KI 0
param set ADRC_PITCH_KI 0
param set ADRC_ROLL_SPS 0
param set ADRC_PITCH_SPS 0
param set ADRC_ROLL_NF 0
param set ADRC_PITCH_NF 0
param set ADRC_ROLL_RAMP 0
param set ADRC_PITCH_RAMP 0

param save
```

**地面验证**：小油门 30-40%，观察机身无剧烈摆动 → 正常起飞。

---

### 第 1 步：CW 8 悬停验证（起点）

**目标**：确认 CW 8 + 硬化项全开 + 姿态环降增益后，飞机能稳定悬停。

**飞法**：低空悬停 30 秒，不打杆。

**判定标准**（看 `listener adrc_status -r 50` 或日志）：

| 指标 | 通过 | 失败 |
|---|---|---|
| `vehicle_rates_setpoint` roll 1~4Hz 峰 | < 5°/s | > 10°/s（姿态环在振荡） |
| `adrc_status.z3` std (roll/pitch) | < 3 rad/s² | > 5 rad/s² |
| `adrc_status.u` 峰值 | > 0.15（有出力） | < 0.08（控制太弱） |
| `adrc_status.e1` RMS | < 0.3 rad/s | > 0.5 rad/s |
| 能爬升到 1m 以上 | 是 | 离地 10cm 就晃（控制太弱） |
| 无 2~4Hz 高频抖动 | 是 | 有（ESO 激振） |

**通过 → 第 2 步。失败 → 见故障处理表。**

---

### 第 2 步：CW 10

```bash
param set ADRC_ROLL_CW 10
param set ADRC_PITCH_CW 10
param set ADRC_YAW_CW 5
param save
```

**飞法**：悬停 30 秒 + 小幅打杆（±10° roll/pitch）。

**判定**：同第 1 步，额外看：
- 打杆后回到悬停的收敛时间（应 < 1 秒）
- 打杆时无 2~4Hz 振铃

**通过 → 第 3 步。出现高频抖 → 加 TAU 到 0.025（不要降 CW）。**

---

### 第 3 步：CW 12

```bash
param set ADRC_ROLL_CW 12
param set ADRC_PITCH_CW 12
param set ADRC_YAW_CW 6
param save
```

**飞法**：悬停 30 秒 + 中幅打杆（±20°）。

**判定**：
- 悬停 RMS 不超过 PID 基线的 1.5 倍
- 打杆响应接近 PID（"跟手"）
- 无 2~4Hz 持续振荡

**通过 → CW 调到位，进入 ESO_W 调整。出现抖动 → 退回 CW 10 或加 TAU 到 0.03。**

---

### 第 4 步：ESO_W 25

```bash
param set ADRC_ROLL_ESO_W 25
param set ADRC_PITCH_ESO_W 25
param set ADRC_YAW_ESO_W 15
param save
```

**目的**：提高扰动估计速度（ESO_W 25 时 z3 跟踪扰动更快，抗风更好）。

**判定**：
- z3 std 应略降（ESO 能看到更多扰动细节）
- 悬停 RMS 不恶化
- 注意 `adrc_status.z3raw` 是否触限（z3raw ≠ z3 → 限幅生效 → ESO_W 太高）

**通过 → 第 5 步。出现高频抖 → 加 TAU 到 0.03 或降 GAMMA 到 0.6。**

---

### 第 5 步：ESO_W 30

```bash
param set ADRC_ROLL_ESO_W 30
param set ADRC_PITCH_ESO_W 30
param set ADRC_YAW_ESO_W 18
param save
```

**目的**：ESO_W 30 + TAU 0.02~0.03 是增广 ESO 的"甜区"（3·30·0.02=1.8>1）。

**判定**：同第 4 步。如果 2~3Hz 抖动出现 → 退回 ESO_W 25，或：
- 优先加 TAU（0.02→0.025→0.03）
- 次选降 GAMMA（0.7→0.6→0.5）
- **不要降 CW**

---

### 第 6 步（可选）：FF/KI 微调

CW/ESO_W 调到位后，如果：
- 打杆跟手不够 → 加前馈：`ADRC_ROLL_FF 0.3` → `0.5`
- 悬停有慢漂/静差 → 加积分：`ADRC_ROLL_KI 0.1` → `0.2`

**注意**：FF 和 KI 必须配对开——FF 引入的静差由 KI 消除。

---

### 第 7 步：PID 对比

ADRC 参数定版后，用**同航线、同环境**飞一组 PID 做对比：

| 指标 | ADRC 目标 | 判定 |
|---|---|---|
| 悬停 RMS (roll/pitch) | ≤ PID × 1.3 | 不差太多即可 |
| 打杆收敛时间 | ≤ PID | |
| z3 std | < 3 rad/s² | ESO 健康 |
| u 能量（抖动成本） | ≤ PID | |

ADRC 的赢面在**抗扰**（风/挂载/重心偏移），不是悬停平滑度。悬停追平 PID 就是成功。

---

## 5. 每次试飞检查清单

| # | 检查项 | 方法 |
|---|---|---|
| 1 | 电池 ≥ 25.0V | QGC 电池栏 |
| 2 | 参数确认 | `param show ADRC_ROLL_CW`（只检查本轮改的） |
| 3 | GPS fix | QGC 状态栏 |
| 4 | 遥控器解锁/紧急按钮可用 | 地面测试 |
| 5 | 小油门地面观察 | 30~40% 油门 5 秒，无剧烈摆动 |
| 6 | 正常起飞 | 低空悬停 |
| 7 | 悬停 30 秒 | 看 QGC 姿态仪是否平稳 |
| 8 | （可选）小幅打杆 | ±10°，看回正速度 |
| 9 | 落地 | 确认日志已记录 |
| 10 | 导出 .ulg | QGC 日志页面 |

---

## 6. 故障识别与处理

### 6.1 1~2Hz 慢摇（"荡秋千"）

**现象**：机身缓慢左右/前后摇摆，频率 1~2Hz，幅度 5~20°。

**原因**：速率环带宽不够，姿态环欠阻尼（即 log_211/212/213 的问题）。

**处理**：
1. 确认 CW/ESO_W 已按上面设置（不是默认值 50/11）
2. 确认电池 ≥ 23.5V（低压 → 推力不足 → 等效 b0 更低）
3. 确认 MC_ROLL_P 已降到 4.5（减少对内环的需求）
4. 如果 CW 已经 12+ 还晃 → **b0 可能偏高**，用 `calibrate_b0.py` 标定后降到真实值
5. 如果以上都做了还晃 → **刷 PID 固件**做对照；PID 也晃 → 机械问题（桨/电机座/IMU 减震）

### 6.2 2~4Hz 高频抖（"嗡嗡响"）

**现象**：机身高频细颤，手摸电机座能感觉到振动，频率 2~4Hz。

**原因**：ESO 带宽扫过执行器/结构谐振，形成极限环（即 log_169/174/190 的问题）。

**处理**（按优先级）：
1. **加 TAU**：0.02 → 0.025 → 0.03（TAU 是防极限环的主力）
2. **降 GAMMA**：0.7 → 0.6 → 0.5（减少扰动补偿强度）
3. **降 ESO_W**：30 → 25 → 20（最后手段，降了抗扰也降）
4. **不要降 CW**——降 CW 让控制更弱，会回到 1~2Hz 慢摇

### 6.3 离地 10cm 就晃/飞不起来

**现象**：油门推到 50~60%，飞机离地 10cm 后剧烈摆动，无法爬升。

**原因**：有效增益 wc/b0 太低（即 log_213 的问题）。地面效应 + 桨流扰动超过控制器能力。

**处理**：
1. 确认 CW ≥ 8（不是默认的 11 或之前的保守值 4）
2. 确认 b0 ≈ 90（不是 200 或 100 的错误值）
3. 确认 MC_ROLL_P ≤ 4.5
4. 确认电池满电
5. **快速推油门**越过地面效应区（0.3~0.5m），不要在 10cm 处悬停

### 6.4 z3 疯狂翻转/发散

**现象**：`adrc_status.z3` 符号快速反转（0.2s 内 ±10+），z3raw ≠ z3（限幅生效）。

**原因**：ESO 发散——ESO_W 太高或 b0 严重失配。

**处理**：
1. 立即降落
2. 降 ESO_W（25→15→10）
3. 检查 b0 标定
4. 加 GAMMA（0.7→0.5）降低补偿强度

### 6.5 打杆就炸

**现象**：悬停平稳，一动杆飞机就剧烈振荡/翻滚。

**原因**：设定值阶跃激励柔性模态。

**处理**：
1. 开设定值平滑：`ADRC_ROLL_SPS 8` / `ADRC_PITCH_SPS 8`
2. 或降低打杆幅度（先小幅验证）
3. 如果不开 SPS 但 CW/ESO_W 已经很高 → 加 RAMP（输出限速）：`ADRC_ROLL_RAMP 10`

---

## 7. b0 标定（悬停稳定后做）

b0 是"归一化扭矩 → 角加速度"的增益，依赖机架惯量、电机推力、电池电压。**SITL 估算值（90/90/8）只当起点，真机必须实测。**

### 方法 1：日志辨识法（推荐，无需额外硬件）

用已有的 ADRC 悬停日志，运行 `calibrate_b0.py`：

```bash
python3 calibrate_b0.py --log your_hover_log.ulg --axis roll
python3 calibrate_b0.py --log your_hover_log.ulg --axis pitch
```

脚本用低频增益法（z3 vs u 回归斜率）反推 b0。

### 方法 2：阶跃法（更准，需安全环境）

1. 解锁、悬停稳定；
2. 在 Acro 模式下给一个已知扭矩阶跃（如 roll +0.2 归一化扭矩，持续 0.5 秒）；
3. 从日志的角速率响应斜率反推：`b0 ≈ Δω̇ / Δu`。

### 方法 3：物理估算法（粗估）

```
b0_roll/pitch ≈ 4 × T_motor × a / (J × u_max)
```
其中 a = 0.233m（半臂长），T_motor ≈ 29.6N（U3515 满油门），J ≈ 0.157 kg·m²，u_max = 1。

**标定后更新参数**：
```bash
param set ADRC_ROLL_B0 <实测值>
param set ADRC_PITCH_B0 <实测值>
param set ADRC_YAW_B0 <实测值>
param save
```

---

## 8. 参数速查表

| 参数 | 含义 | 起点值 | 调参方向 | 备注 |
|---|---|---|---|---|
| `ADRC_*_CW` | 控制带宽 ωc (rad/s) | 8/8/5 | ↑ 更跟手，↓ 更稳 | 与 ESO_W 保持 2~3× |
| `ADRC_*_ESO_W` | 观测器带宽 ωo (rad/s) | 20/20/12 | ↑ 抗扰更快，↓ 更稳 | 需 3·ESO_W·TAU>1 |
| `ADRC_*_B0` | 控制增益 | 90/90/8 | 标定后用实测值 | 偏大→z3 偏大，偏小→控制弱 |
| `ADRC_*_TAU` | 执行器延时 (s) | 0.02/0.02/0.01 | ↑ 更防激振 | 14"桨典型 0.01~0.03 |
| `ADRC_*_GAMMA` | 扰动补偿系数 | 0.7/0.7/0.6 | ↓ 更稳（抗扰降） | 0.5~0.9 |
| `ADRC_*_FLT` | 反馈低通 (Hz) | 30/30/20 | ↓ 更平滑（相位滞后） | 20~50 |
| `ADRC_*_Z3MAX` | z3 限幅 (rad/s²) | 0(自动) | 手动值<自动值→更紧 | 0 = 2·b0 |
| `ADRC_*_FF` | 前馈 | 0 | 0.3~0.6 | 需配 KI |
| `ADRC_*_KI` | 积分 | 0 | 0.1~0.3 | 需配 FF |
| `ADRC_*_SPS` | 设定值平滑 (Hz) | 0 | 5~10（打杆炸时开） | |
| `ADRC_*_RAMP` | 输出限速 (1/s) | 0 | 5~20（必要时） | |
| `ADRC_*_NF` | 反馈陷波 (Hz) | 0 | FFT 峰值（最后手段） | 低频 notch 有相位风险 |
| `MC_ROLL_P` | 姿态环增益 | 4.5 | ↑ 更快（需内环跟得上） | 原值 5.65 |
| `MC_AIRMODE` | 悬停油门权威 | 1 | — | 0→1 免费提升 |

---

## 9. 调参流程图

```
起点（CW 8, ESO_W 20, TAU 0.02, γ 0.7, FLT 30, P_att 4.5）
  │
  ├─ 悬停 30s ──→ 通过？
  │     │                │
  │     │ 否             │ 是
  │     ▼                ▼
  │  1~2Hz 慢摇？    CW → 10 → 12
  │     │                │
  │     │ 是             ▼
  │     ▼            ESO_W → 25 → 30
  │  ↑CW / ↓P_att        │
  │                       ▼
  │                   打杆响应 OK？
  │                     │    │
  │                     │ 否  │ 是
  │                     ▼     ▼
  │                  +FF/KI  PID 对比
  │                           │
  │                           ▼
  │                       b0 标定
  │                           │
  │                           ▼
  │                      调参完成
  │
  ├─ 2~4Hz 高频抖？
  │     │
  │     ▼
  │  +TAU (0.02→0.03)
  │  ↓GAMMA (0.7→0.6)
  │  ↓ESO_W（最后）
  │  不要 ↓CW
  │
  └─ z3 发散？
        │
        ▼
     ↓ESO_W / ↓GAMMA / 标定 b0
```

---

## 10. 数据分析工具

### 10.1 实时监听（飞行中/地面）

```
listener adrc_status -r 50       # 每 20ms 刷新一次 ADRC 内部状态
listener vehicle_torque_setpoint # 输出力矩
listener vehicle_rates_setpoint  # 姿态环下发的速率指令
```

关注字段：
- `z1`：实际角速率估计
- `z3`：总扰动估计（std < 3 为健康）
- `z3raw`：限幅前的 z3（z3raw ≠ z3 → 限幅生效）
- `u`：控制输出（应随误差成比例，不是顶死在 0.08）
- `y`：滤波后的反馈（应比原始角速率平滑）
- `ueso`：增广模式下 = 执行器扭矩状态估计

### 10.2 日志分析（落地后）

```bash
# 提取 ADRC 参数确认
ulog_params your_log.ulg | grep ADRC

# 快速角速率统计
python3 -c "
from pyulog import ULog; import numpy as np
u = ULog('your_log.ulg')
d = u.get_dataset('vehicle_angular_velocity')
t = (d.data['timestamp']-d.data['timestamp'][0])*1e-6
armed = ...  # 需要过滤解锁段
for i,nm in [(0,'roll'),(1,'pitch'),(2,'yaw')]:
    x = d.data['xyz[%d]'%i]
    print('%s: RMS %.2f deg/s' % (nm, np.degrees(np.std(x))))
"
```

### 10.3 FFT 找谐振峰

```python
import numpy as np
from pyulog import ULog
u = ULog('your_log.ulg')
d = u.get_dataset('vehicle_angular_velocity')
t = (d.data['timestamp']-d.data['timestamp'][0])*1e-6
x = d.data['xyz[0]']  # roll rate
# 取悬停段
dt = np.median(np.diff(t)); fs = 1/dt
x = x - np.mean(x); n = len(x)
win = np.hanning(n)
X = np.fft.rfft(x * win)
fr = np.fft.rfftfreq(n, dt)
mag = 2*np.abs(X)/np.sum(win)
# 找 0.5~10Hz 峰
mask = (fr>=0.5)&(fr<=10)
peak = fr[mask][np.argmax(mag[mask])]
print('主峰: %.2f Hz' % peak)
```

---

## 11. 调参记录模板

每次试飞填写：

```
飞行编号：___
日期/时间：___
电池电压（起飞前）：___V
本轮改动：param set ___ ___
其他参数（不变的列出关键值）：CW=___, ESO_W=___, b0=___, TAU=___, γ=___

飞行时长：___s
悬停 RMS (roll/pitch/yaw)：___/___/___  °/s
z3 std (roll/pitch)：___/___  rad/s²
u 峰值 (roll/pitch)：___/___
e1 RMS (roll/pitch)：___/___  rad/s
z3raw ≠ z3 次数：___
FFT 主峰频率：___Hz（应为 <2Hz 或无尖峰）
是否出现 2~4Hz 抖动：是/否
是否能稳定悬停：是/否
打杆响应（如有）：跟手/偏软/振铃

结论：通过/退回上一组/需调整其他参数
下一步：___
```

---

## 12. 预期管理

ADRC 在这台真机上的**现实目标**：

| 指标 | 目标 | 不要期待 |
|---|---|---|
| 悬停平滑度 | 追平 PID（±30% 以内） | 远超 PID |
| 抗风/抗扰 | 优于 PID | — |
| 参数数量 | 74 个（多于 PID 的 ~10 个） | 比 PID 简单 |
| 调参周期 | 比 PID 长 2~3 倍 | 比 PID 快 |

**一句话**：如果 ADRC 能做到"悬停和 PID 一样稳，刮风时比 PID 更稳"，这个项目就是成功的。
