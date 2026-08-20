#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ADRC 内环 b0 标定脚本 v2 — 闭环数据修正版

用法:
    python3 calibrate_b0_v2.py <flight.ulg> [--b0-used 100 100 50] [--wc 8] [--gamma 0.7]

核心限制:
    悬停数据的激励太弱, 所有辨识方法都有较大不确定性。
    本脚本给出的是"粗略范围 + 定性诊断", 不是精确值。
    最可靠的 b0 标定: Acro 模式 + 已知力矩阶跃 + 测量角加速度斜率。

方法:
    ✓ IV 频响: 以 sp 为工具变量的 Welch 互谱 (闭环偏置最小, 但悬停中 sp 变化小 → 噪声大)
    ✓ 闭环传递函数: 测量 sp→ω 的闭环带宽, 利用已知控制器结构反推 (需要清晰的 -3dB 点)
    ✓ z3-u 相关性: 定性判断 b0_used 是否接近 b0_true (相关性强 → b0 偏差大)
    ⚠️ 传统回归: 闭环有偏, 系统性偏低
"""

import sys
import argparse
import numpy as np
from pyulog import ULog


def get(ulog, name):
    for m in ulog.data_list:
        if m.name == name:
            return m
    raise KeyError(f"topic {name} not in log")


def welch_csd(x, y, fs, nperseg=1024):
    n = min(nperseg, len(x) // 4)
    if n < 64:
        return None, None, None, None
    nseg = len(x) // n
    if nseg < 1:
        return None, None, None, None
    S_xy = np.zeros(n // 2 + 1, complex)
    S_xx = np.zeros(n // 2 + 1, complex)
    S_yy = np.zeros(n // 2 + 1, complex)
    win = np.hanning(n)
    for s in range(nseg):
        sl = slice(s * n, (s + 1) * n)
        X = np.fft.rfft((x[sl] - x[sl].mean()) * win)
        Y = np.fft.rfft((y[sl] - y[sl].mean()) * win)
        S_xy += X.conj() * Y
        S_xx += X.conj() * X
        S_yy += Y.conj() * Y
    freqs = np.fft.rfftfreq(n, 1 / fs)
    return freqs, S_xy / nseg, S_xx / nseg, S_yy / nseg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="path to .ulg")
    ap.add_argument("--b0-used", nargs=3, type=float, default=[100.0, 100.0, 50.0],
                    help="当前固件里使用的 b0 (roll pitch yaw)")
    ap.add_argument("--wc", type=float, default=8.0, help="控制器带宽 wc (rad/s)")
    ap.add_argument("--gamma", type=float, default=0.7, help="扰动补偿系数 gamma")
    args = ap.parse_args()

    ulog = ULog(args.log)
    t0 = ulog.start_timestamp
    u_m = get(ulog, "vehicle_torque_setpoint")
    w_m = get(ulog, "vehicle_angular_velocity")
    sp_m = get(ulog, "vehicle_rates_setpoint")
    status = get(ulog, "vehicle_status")

    tu = (np.array(u_m.data["timestamp_sample"]) - t0) / 1e6
    tw = (np.array(w_m.data["timestamp_sample"]) - t0) / 1e6
    tsp = (np.array(sp_m.data["timestamp"]) - t0) / 1e6
    t_st = (np.array(status.data["timestamp"]) - t0) / 1e6
    armed = np.array(status.data["arming_state"]) == 2
    arm_interp = np.interp(tu, t_st, armed.astype(float)) > 0.5

    try:
        a_m = get(ulog, "adrc_status")
    except KeyError:
        a_m = None

    fs = 1.0 / np.median(np.diff(tu)) if len(tu) > 1 else 100.0
    names = ["roll", "pitch", "yaw"]

    print(f"log: {args.log}")
    print(f"时长: {(ulog.last_timestamp - ulog.start_timestamp)/1e6:.1f}s, fs={fs:.0f}Hz")
    print(f"b0_used: {args.b0_used[0]:.0f}/{args.b0_used[1]:.0f}/{args.b0_used[2]:.0f}, wc={args.wc}, γ={args.gamma}")

    for i, nm in enumerate(names):
        u_arr = np.array(u_m.data[f"xyz[{i}]"])
        wd = np.interp(tu, tw, np.array(w_m.data[f"xyz_derivative[{i}]"]))
        rate = np.interp(tu, tw, np.array(w_m.data[f"xyz[{i}]"]))
        sp = np.interp(tu, tsp, np.array(sp_m.data[["roll", "pitch", "yaw"][i]]))

        u_a = u_arr[arm_interp]; wd_a = wd[arm_interp]
        rate_a = rate[arm_interp]; sp_a = sp[arm_interp]

        print(f"\n{'='*60}")
        print(f" {nm} 轴 (b0_used={args.b0_used[i]:.0f})")
        print(f"{'='*60}")

        # --- z3-u 相关性 (定性诊断) ---
        if a_m is not None:
            ta = (np.array(a_m.data["timestamp"]) - t0) / 1e6
            z3 = np.array(a_m.data[f"z3[{i}]"])
            uu_z = np.interp(ta, tu, u_arr)
            # 取解锁段
            arm_a = np.interp(ta, t_st, armed.astype(float)) > 0.5
            if np.sum(arm_a) > 100:
                corr = np.corrcoef(z3[arm_a], uu_z[arm_a])[0, 1]
                z3_std = np.std(z3[arm_a])
                if abs(corr) < 0.15:
                    diag = "✓ z3-u 相关性弱 → b0_used 可能接近真实值"
                elif abs(corr) < 0.35:
                    diag = "△ z3-u 有中等相关 → b0 可能有 20-40% 偏差"
                else:
                    diag = "⚠️ z3-u 强相关 → b0 有明显偏差"
                print(f"  {diag}")
                print(f"     (相关系数={corr:.3f}, z3 std={z3_std:.2f})")
                if corr > 0.15:
                    # z3 = (b0_true - b0_used)·u + f, 正相关 → b0_true > b0_used
                    print(f"     → b0_true 可能 > b0_used ({args.b0_used[i]:.0f}), 建议增大 b0")
                elif corr < -0.15:
                    print(f"     → b0_true 可能 < b0_used ({args.b0_used[i]:.0f}), 建议减小 b0")

        # --- IV 频响 ---
        res2 = welch_csd(u_a, sp_a, fs, nperseg=min(1024, len(u_a)//4))
        res3 = welch_csd(wd_a, sp_a, fs, nperseg=min(1024, len(wd_a)//4))
        if res2[0] is not None and res3[0] is not None:
            fr2, S_usp, S_uu, S_spsp = res2
            _, S_wdsp, _, _ = res3
            band_iv = (fr2 >= 0.3) & (fr2 <= 1.5)
            if np.any(band_iv):
                b0_iv = S_wdsp[band_iv] / (S_usp[band_iv] + 1e-30)
                weight = np.abs(S_spsp[band_iv])
                if weight.sum() > 0:
                    iv_gain = np.real(np.sum(b0_iv * weight) / weight.sum())
                    coh = np.abs(S_usp[band_iv])**2 / (np.abs(S_uu[band_iv]) * np.abs(S_spsp[band_iv]) + 1e-30)
                    coh_mean = np.mean(coh)
                    q = "✓" if coh_mean > 0.4 else "△"
                    print(f"  {q} IV 频响: b0 ≈ {iv_gain:.0f}  (0.3~1.5Hz, 相干性={coh_mean:.2f})")
                    if coh_mean < 0.3:
                        print(f"     ⚠️ 相干性低, 估计不可靠 — 悬停中 sp 变化太小")

        # --- 闭环传递函数 ---
        res4 = welch_csd(sp_a, rate_a, fs, nperseg=min(2048, len(sp_a)//4))
        if res4[0] is not None:
            fr4, S_sprate, S_spsp4, _ = res4
            H_mag = np.abs(S_sprate / (S_spsp4 + 1e-30))
            dc_band = (fr4 >= 0.2) & (fr4 <= 0.5)
            dc_gain = np.mean(H_mag[dc_band]) if np.any(dc_band) else 1.0
            threshold = dc_gain / np.sqrt(2)
            band_fit = (fr4 >= 0.3) & (fr4 <= 3.0)
            below = H_mag[band_fit] >= threshold
            if np.any(below) and not np.all(below):
                indices = np.where(band_fit)[0]
                below_mask = H_mag[indices] >= threshold
                last_above = indices[below_mask][-1]
                if last_above + 1 < len(fr4):
                    f1, f2 = fr4[last_above], fr4[last_above + 1]
                    h1, h2 = H_mag[last_above], H_mag[last_above + 1]
                    omega_3db = f1 + (threshold - h1) / (h2 - h1) * (f2 - f1) if h1 != h2 else f1
                    omega_3db_rad = omega_3db * 2 * np.pi
                    wc = args.wc; gamma = args.gamma; b0_used = args.b0_used[i]
                    denom = wc - gamma * omega_3db_rad
                    if denom > 0.1:
                        b0_cl = omega_3db_rad * (1 - gamma) * b0_used / denom
                        print(f"  ✓ 闭环带宽法: DC={dc_gain:.3f}, -3dB={omega_3db:.2f}Hz → b0 ≈ {b0_cl:.0f}")
                    else:
                        print(f"  ✗ 闭环带宽法: -3dB={omega_3db:.2f}Hz, 分母过小")
            else:
                print(f"  ✗ 闭环带宽法: 3Hz 内未降到 -3dB (DC增益={dc_gain:.3f})")

        # --- 传统回归 (有偏) ---
        slopes = []
        for w0 in np.arange(0, len(u_a)/fs - 1.0, 0.5):
            idx = int(w0 * fs); idx2 = int((w0 + 1.0) * fs)
            if idx2 > len(u_a): break
            uu = u_a[idx:idx2]; ww = wd_a[idx:idx2]
            if len(uu) < 10 or uu.std() < 0.02: continue
            A = np.vstack([uu, np.ones_like(uu)]).T
            s = np.linalg.lstsq(A, ww, rcond=None)[0][0]
            if np.isfinite(s) and 0 < s < 500: slopes.append(s)
        reg = np.median(slopes) if slopes else float('nan')
        print(f"  ⚠️ 直接回归: b0 ≈ {reg:.0f}  (闭环偏低, {len(slopes)} 窗口)")

    # --- 推荐做法 ---
    print(f"\n{'='*60}")
    print("结论与推荐做法:")
    print(f"{'='*60}")
    print()
    print("  悬停数据激励不足, 所有方法的 b0 估计都有较大不确定性。")
    print("  最可靠的标定方法是专门的激励测试:")
    print()
    print("  方法: Acro 模式悬停 → 给一个已知力矩阶跃 (如 roll +0.1)")
    print("        → 从日志中读取角加速度响应斜率 → b0 = Δω̇ / Δu")
    print()
    print("  或者: 直接用你飞过的日志对比 — 哪个 b0 更稳就用哪个。")
    print("  你已经验证: b0=100 > b0=80 > b0=90 (稳定性)。")
    print("  → 建议 b0=100 或 b0=90, 不要再往低了调。")
    print()
    print("  z3-u 相关性是最简单的定性检查:")
    print("  相关性强 → b0 偏差大; 相关性弱 → b0 接近。")


if __name__ == "__main__":
    main()
