#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ADRC 内环 b0 标定脚本（基于 PX4 ulog 飞行日志）

用法:
    python3 calibrate_b0.py <flight.ulg> [--b0-used 100 100 50]

输入日志需要包含:
    vehicle_torque_setpoint   (u, 归一化力矩)
    vehicle_angular_velocity  (xyz_derivative, 滤波后角加速度)

输出:
    每轴三种 b0 估计:
      1) 窗口线性回归   wdot = b0*u + c   (1s 窗口, 取斜率中位数)
      2) 低频频响增益   |H(wdot/u)| @ 0.2~2 Hz (Welch 互谱, 最推荐)
      3) z3 反推         z3 = a*u + b  =>  b0_true ≈ b0_used + a

注意: 闭环数据三种方法都有偏置, 请以"频响低频增益"为主, 多段取中位数;
      并在悬停+单轴激励(双脉冲/扫频)的日志上使用, 不要用爬升/急转弯段。
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="path to .ulg")
    ap.add_argument("--b0-used", nargs=3, type=float, default=[100.0, 100.0, 50.0],
                    help="当前固件里使用的 b0 (roll pitch yaw)")
    args = ap.parse_args()

    ulog = ULog(args.log)
    t0 = ulog.start_timestamp
    u_m = get(ulog, "vehicle_torque_setpoint")
    w_m = get(ulog, "vehicle_angular_velocity")
    try:
        a_m = get(ulog, "adrc_status")
    except KeyError:
        a_m = None

    tu = (np.array(u_m.data["timestamp_sample"]) - t0) / 1e6
    tw = (np.array(w_m.data["timestamp_sample"]) - t0) / 1e6
    names = ["roll", "pitch", "yaw"]

    print(f"log: {args.log}")
    print(f"时长: {(ulog.last_timestamp - ulog.start_timestamp)/1e6:.1f} s")
    print(f"当前 b0 (used): roll={args.b0_used[0]} pitch={args.b0_used[1]} yaw={args.b0_used[2]}")

    for i, nm in enumerate(names):
        u = np.array(u_m.data[f"xyz[{i}]"])
        wd = np.interp(tu, tw, np.array(w_m.data[f"xyz_derivative[{i}]"]))
        fs = 1.0 / np.median(np.diff(tu)) if len(tu) > 1 else 100.0

        # --- 方法1: 窗口线性回归 ---
        slopes = []
        win = 1.0
        for w0 in np.arange(tu[0], tu[-1] - win, win * 0.5):
            mask = (tu >= w0) & (tu < w0 + win)
            if mask.sum() < 10:
                continue
            uu, ww = u[mask], wd[mask]
            if uu.std() < 0.02:
                continue
            A = np.vstack([uu, np.ones_like(uu)]).T
            s = np.linalg.lstsq(A, ww, rcond=None)[0][0]
            if np.isfinite(s) and abs(s) < 500:
                slopes.append(s)
        slopes = np.array(slopes)
        reg = np.median(slopes) if len(slopes) else float("nan")

        # --- 方法2: 低频频响增益 (Welch) ---
        n = min(1024, len(u) // 4)
        nseg = len(u) // n
        S_uu = np.zeros(n // 2 + 1, complex)
        S_wu = np.zeros(n // 2 + 1, complex)
        winf = np.hanning(n)
        for s in range(nseg):
            a = (u[s * n:(s + 1) * n])
            b = (wd[s * n:(s + 1) * n])
            U = np.fft.rfft((a - a.mean()) * winf)
            W = np.fft.rfft((b - b.mean()) * winf)
            S_uu += U.conj() * U
            S_wu += W * U.conj()
        f = np.fft.rfftfreq(n, 1 / fs)
        band = (f >= 0.2) & (f <= 2.0)
        if S_uu[band].sum() > 0:
            freq_gain = np.abs(S_wu[band]).sum() / S_uu[band].sum()
        else:
            freq_gain = float("nan")

        # --- 方法3: z3 反推 (需要 adrc_status) ---
        z3fit = float("nan")
        if a_m is not None:
            ta = (np.array(a_m.data["timestamp"]) - t0) / 1e6
            z3 = np.array(a_m.data[f"z3[{i}]"])
            uu = np.interp(ta, tu, u)
            A = np.vstack([uu, np.ones_like(uu)]).T
            a = np.linalg.lstsq(A, z3, rcond=None)[0][0]
            z3fit = args.b0_used[i] + a

        print(f"\n{nm} 轴:")
        print(f"  窗口回归  b0 ≈ {reg:7.1f}   (窗口数={len(slopes)})")
        print(f"  低频增益  b0 ≈ {freq_gain:7.1f}   (0.2~2 Hz, 推荐)")
        if np.isfinite(z3fit):
            print(f"  z3 反推   b0 ≈ {z3fit:7.1f}")
        print(f"  建议      取频响与回归的中间值; 若三种差异大, 做一次更充分的单轴激励飞行")

    print("\n判定: 设好 b0 后, 机动中 z3 幅值应明显小于 b0*u 的量级;")
    print("      若 z3 与 u 强相关且大 -> b0 仍偏, 按差值方向修正后再飞一次。")


if __name__ == "__main__":
    main()
