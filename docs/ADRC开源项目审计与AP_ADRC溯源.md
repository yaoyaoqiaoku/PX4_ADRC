# ADRC 开源项目审计与 AP_ADRC 溯源

> 日期：2026-08-13
> 目的：回答“参考了哪些开源项目、思路是什么、AP_ADRC 到底参考没有”
> 立场声明：本报告严格区分【深度参考】【结构参考】【目录级检查】【本轮新查到】四档，不夸大。

---

## 0. 结论先行（诚实版）

1. **你给的 10 个仓库：全部检查过**，其中 4 个是深度参考，其余是目录/结构级检查。
2. **自己搜的项目：参考了 5 类**，最有价值的是 FMT（阿木）ADRC —— 它是目前国内唯一有**真机量产验证**的开源飞控 ADRC。
3. **AP_ADRC：之前文档提过名字，但没有真正读到源码——这是疏漏，这一轮补齐了。** 经过溯源，AP_ADRC 的真实身份是 ArduPilot 的一个 **Draft PR（#20243，2022-03，从未合并）**，改编自另一个同样未合并的 PR（#20079）。它不是主线代码，也没有大量真机测试证据。说它是“飞控里最成熟的 ADRC 实现”**不准确**。
4. AP_ADRC 的溯源反而带来了两条**直接可用**的真机优化线索（详见第 5 节）。

---

## 1. 你给的开源项目：参考情况与思路

| 仓库 | 参考深度 | 从中得到的思路 | 落地到我们模块哪里 |
|---|---|---|---|
| zhaohaojie1998/Control-Algorithm | 深度（你模块的原型） | 经典二阶 ADRC：TD + ESO + NLSEF，b0/wc 参数化；教学代码风格，缺真机硬化 | 模块最初就是从它移植的 |
| zzqzzqzzq2002/ThisisADRC | 深度（C 库逐函数读） | ESO 状态必须饱和防发散；NLSEF 带积分项可补 b0 失配；TD 的 fhan 离散实现 | 已实现 z3 限幅（Z3MAX）+ KI 条件积分 |
| Cking616/Adavance_PID | 深度（教材实现） | 《先进PID控制》配套代码，fal/ESO 标准写法，适合对照离散化细节 | 校验 fal 与 ESO 离散公式 |
| kbmajeed/adaptive_adrc | 结构参考（MATLAB 高度环） | LMS 在线自适应 ESO 增益 → b0 标定不准时自动修正 | 列为后续方向：b0 在线辨识 |
| htchit/PX4_ROS_ADRC | 结构参考 | ADRC 跑在机载电脑，MAVROS 下发 → 端到端延时 10~30 ms | 印证：角速率环必须留在飞控内，机载 ADRC 只适合慢环 |
| SWUST-ICAA/px4adrc | 结构参考 | 同上，机外 ADRC 方案 | 同上 |
| ljr040210/adrc_px4 | 目录级 | PX4 fork 内含 ADRC 模块，结构与我们类似（替换 rate control） | 确认“替换速率环”是主流做法 |
| Sunyi16/PX4-Morphing | 目录级 | PX4 fork，姿态/速率环改造 | 结构参考 |
| iMinloha/PX4-Autopilot | 目录级 | PX4 fork，ADRC 相关改动 | 结构参考 |
| mingxuZhang2/ADRC | 目录级 | 经典 ADRC 实现 | 对照用 |

**这一轮的核心思路**：你的 10 个仓库绝大多数是“把 ADRC 塞进飞控”的教学/实验代码，**没有任何一个解决了真机振荡的三个根因**（反馈噪声、执行器延时、ESO 发散）。所以 v2 方案没有照抄任何一个，而是把它们的共性（b0/wc/wo 参数化、z3 饱和、积分项）提取出来，加上真机硬化层。

---

## 2. 自己搜的开源项目：参考情况与思路

### 2.1 FMT（阿木）ADRC —— 最重要的参考

- 来源：Firmament-Autopilot/FMT-Firmware，Simulink 模型生成，已合并主仓库；MFP450-ADRC 套件真机发售
- 关键发现（360doc《自抗扰控制在FMT飞控上的应用》）：

  > 引入扰动补偿系数 gamma ∈ [0,1]，控制扰动补偿的大小。实际应用中由于存在较大的观测噪声以及系统延迟，**完全补偿总扰动可能导致系统不稳定甚至震荡**，通过降低补偿比例提高鲁棒性。

- 这正是你 2.4~3 Hz 振荡的官方“教科书级”解释。FMT 的解法是**不完全补偿**，我们的解法是 **TAU 延时建模 + z3 限幅 + 反馈滤波**，思路同源、手段不同。
- 结论：我们 v2 的方向和 FMT 真机验证过的方向一致。

### 2.2 DevangPatwardhan/SMC-PX4

- 用滑模控制替换 PX4 速率环 PID。结论：任何“高级控制器”替换速率环，都必须先过滤波 + 延时这两关——和我们的诊断互相印证。

### 2.3 无名科创（wustyuyi/NamelessCotrunQuad_V1.0）

- 国内知名开源飞控，只在**角速度环**用 ADRC，角度环只用 P。印证我们的替换位置正确。

### 2.4 ACFLY

- “单参数对模型 ADRC” + 反步控制。主打调参简单，但未开源完整代码（只有介绍），仅参考思路。

### 2.5 Jiachi Zou 2018 硕士论文（TU/e）

- 《Robust ADRC Scheme for Quadrotor UAVs: Experimental Prototyping》—— AP_ADRC 的理论出处，也是 FMT 补偿系数的出处。
- 论文明确考虑**测量噪声 + 系统延迟**，提出鲁棒化 ADRC 结构。后续应精读并落地。

---

## 3. AP_ADRC 溯源（本轮重点）

### 3.1 真实身份

- **不是 ArduPilot 主线代码**。它是 GitHub 上 ArduPilot/ardupilot 的 **Draft Pull Request #20243**：
  - 作者：MichelleRos，2022-03-07 创建
  - 标题：*ADRC: Active Disturbance Rejection Control*
  - 说明：*“This adds the ADRC controller as an option for Copter attitude control.”*
  - 改编自 xianglunkai 的 **PR #20079**（2022-02，已关闭）
  - 状态：Draft、从未合并、后来停滞
- tridge（ArduPilot 维护者）在 #20079 下要求提供测试证据；在 #20243 下说 *“this is just an experiment, it would be structured quite differently if we were to consider it for merging into master”*。

### 3.2 代码结构（依据 PR 页面 + CSDN《ADRC Ardupilot代码分析》全文）

- 新增库：`libraries/AP_ADRC/AP_ADRC.h` / `AP_ADRC.cpp`
- 集成点：
  - `wscript` 增加 AP_ADRC 编译项
  - `RC_Channel.cpp` 增加 ALT_RATE_CONTROL（高度变化率控制）
  - `AC_AttitudeControl_Multi.h/.cpp` 增加 `_pid2_rate_roll/_pitch/_yaw` 三套并行控制器（PID / ADRC 可选）
- 参数（AP_GROUPINFO，除 _order 为 AP_Int8 外均为 AP_Float）：
  - `_wc`：控制带宽 rad/s
  - `_wo`：ESO 带宽 rad/s
  - `_b0`：控制输入增益
  - `_delta`：fal 线性区长度
  - `_order`：控制模式（两种非线性模式）
  - `_limit`：输出限幅
- ESO 状态：`_z1`（输出观测）、`_z2`（微分观测）、`_z3`（总扰动）
- 误差定义：控制器误差 = 目标 − z1；微分误差 = −z2；观测误差 = z1 − 实测
- 函数：`fal()`（非线性）、`sign()`、`reset_eso()`（z1=实测，z2=z3=0）
- 理论依据：PR 内附 Jiachi Zou 论文 PDF（github files/8202606/Jiachi_Zou_Thesis.2018.ADRC.UAV.Github.pdf）

### 3.3 作者本人的关键评价

PR #20243 评论区，论文作者 **JcZou 亲自回复**：

> *“In yours implementation, fal non-linear control law is used, which is standard method introduced by ADRC. However, based on my experience, a well-tuned PID + ESO would get a better performance, as fal is relatively too sensitive for the error.”*

翻译：fal 非线性控制律对误差太敏感，**一个调好的 PID + ESO 反而效果更好**。

这条评论直接支持我们 v2 的选择：**LADRC（线性 PD 控制律 + 线性 ESO）+ 真机硬化**，而不是照抄 fal 非线性版。

### 3.4 为什么之前没有拿到

- AP_ADRC 不在 ArduPilot 主线，也不在任何活跃 fork 里；grep.app / GitHub 代码搜索 / Software Heritage 均无索引或需要登录
- 只通过 CSDN 分析文章能间接看到结构，无法直接获取源码文件
- 本轮通过 PR 页面拿到了关键信息（结构、参数、作者评价），源码正文仍受网络限制未能整份拉取

---

## 4. 与我们的实现的对比

| 维度 | AP_ADRC (PR #20243) | 我们的 adrc_rate_control (v2) |
|---|---|---|
| 平台 | ArduPilot Copter 4.x（draft） | PX4 1.15.4（已编译 micoair + fmu-v6c） |
| 控制律 | 非线性 fal（2 种 order 模式） | 可选：线性 PD（默认）/ 非线性 fal |
| ESO | 非线性 fal ESO，z1/z2/z3 | 可选线性/非线性 ESO，带宽参数化 |
| 参数 | wc / wo / b0 / delta / order / limit | + FLT 反馈滤波 + NF/NBW 陷波 + TAU 执行器延时 + Z3MAX 限幅 + FF 前馈 + KI 积分 + RAMP 限速 |
| 真机硬化 | 无（实验性质） | 有（针对振荡三根因） |
| 真机验证 | 无公开测试数据（tridge 要求补证据） | SITL + 真机日志迭代（仍在优化） |
| z3 防发散 | 无 | 有（Z3MAX 自动 2\|b0\|） |
| 执行器延时 | 无 | 有（TAU 一阶模型） |

**结论**：AP_ADRC 证明了“b0/wc/wo 三参数化 + 替换速率环”是社区共识；但它缺的正是我们 v2 补上的东西。它的价值在于**对照**，不在于照抄。

---

## 5. 从 AP_ADRC 溯源得到的真机优化线索（下一步）

### 5.1 增加“扰动补偿比例”参数（借鉴 FMT gamma / Jiachi Zou 论文）

- 控制律从 `u = (u0 - z3)/b0` 改为 `u = (u0 - gamma*z3)/b0`
- gamma ∈ [0,1]，默认 1.0（完全补偿，等于现在行为）
- 真机振荡时从 gamma=0.5~0.7 起步，配合 ESO_W 降低，通常能直接压掉极限环
- 物理意义：不追求“完全抗扰”，换取鲁棒性——这正是 FMT 真机验证过的路线

### 5.2 把 fal 的 delta 纳入可调（已有部分基础）

- 我们有 CTL_LAW 切换，可再补一个 fal 的 alpha/delta 整定入口，方便复现 AP_ADRC 行为做 A/B 对比

### 5.3 参考 JcZou 的“PID + ESO”路线

- 论文作者自己建议的：外环用调好的 PID 特性，ESO 只负责补偿低频扰动
- 对应我们模块：FF（前馈）+ KI（积分）已经具备，可以做成“低 CW + 高 FF + ESO 只补偿慢变扰动”的配置

---

## 6. 附：AP_ADRC 关键链接

- Draft PR：https://github.com/ArduPilot/ardupilot/pull/20243
- 原始 PR（已关闭）：https://github.com/ArduPilot/ardupilot/pull/20079
- 理论论文（TU/e 开放获取）：https://pure.tue.nl/ws/portalfiles/portal/110035542/Jiachi_Zou_Thesis.pdf
- 论文作者真机测试代码：https://github.com/JcZou/StarryPilot/blob/master/starry_fmu/Framework/source/Control/adrc_att.c
- 论文作者测试视频：https://www.youtube.com/watch?v=77-_nF-qqpA&t=191s
- CSDN 结构分析：https://blog.csdn.net/qq_26550927/article/details/127804649
- FMT 补偿系数文章：http://www.360doc.com/content/23/1011/22/70238708_1099845542.shtml

---

## 7. 落地状态（2026-08-13 晚）

审计结论已转化为代码迭代（v3），全部编译通过：

- **gamma 扰动补偿系数** `ADRC_*_GAMMA`（FMT/Jiachi Zou 论文）→ 已实现，默认 1.0；
- **设定值平滑** `ADRC_*_SPS`（AP_ADRC/无名/ACFLY 的 TD 思想简化版）→ 已实现，默认 0=关；
- **调试字段** `z3raw`（限幅前 z3）与 `sp`（实际设定值）→ 已加入 adrc_status；
- 编译：SITL ✅ / micoair h743-v2 ✅ 88.84% / fmu-v6c ✅ 93.87%；
- 单元测试：gamma 补偿单调性、SPS 平滑收敛、z3 显式/自动限幅 ✅ PASS。

真机推荐起点与逐步放开顺序见 `params_adrc_real.txt`（v3 版）。
