#!/usr/bin/env python3
"""Deep analysis of the 3 real-drone ADRC logs."""
import numpy as np
from pyulog import ULog

LOGS = [
    ('log_211', '/home/liuyao/下载/rizhi/log_211_2026-8-12-16-21-54.ulg'),
    ('log_212', '/home/liuyao/下载/rizhi/log_212_2026-8-13-16-32-56.ulg'),
    ('log_213', '/home/liuyao/下载/rizhi/log_213_2026-8-13-16-34-02.ulg'),
]

def ds(ulog, name):
    for d in ulog.data_list:
        if d.name == name:
            return d
    return None

def tsec(d):
    return (np.asarray(d.data['timestamp']) - d.data['timestamp'][0]) * 1e-6

def interp(t_new, t_old, y_old):
    return np.interp(t_new, t_old, y_old)

def fft_peak(t, x, fmin=0.5, fmax=12.0):
    """Return (freq_hz, amplitude) of the dominant peak in [fmin,fmax]."""
    t = t - t[0]
    dt = np.median(np.diff(t))
    x = x - np.mean(x)
    n = len(x)
    if n < 64:
        return np.nan, np.nan
    win = np.hanning(n)
    X = np.fft.rfft(x * win)
    freqs = np.fft.rfftfreq(n, dt)
    mag = 2.0 * np.abs(X) / np.sum(win)
    mask = (freqs >= fmin) & (freqs <= fmax)
    if not np.any(mask):
        return np.nan, np.nan
    idx = np.argmax(mag[mask])
    return freqs[mask][idx], mag[mask][idx]

def analyze(name, fname):
    print('=' * 100)
    print(f'### {name}  ({fname.split("/")[-1]})')
    u = ULog(fname)
    vad = ds(u, 'vehicle_angular_velocity')
    adrc = ds(u, 'adrc_status')
    status = ds(u, 'vehicle_status')
    land = ds(u, 'vehicle_land_detected')
    lpos = ds(u, 'vehicle_local_position')
    vtq = ds(u, 'vehicle_torque_setpoint')
    bat = ds(u, 'battery_status')

    t_status = tsec(status)
    armed = np.asarray(status.data['arming_state']) == 2
    nav = np.asarray(status.data['nav_state'])
    t_land = tsec(land)
    landed = np.asarray(land.data['landed'])
    t_lpos = tsec(lpos)
    alt = -np.asarray(lpos.data['z'])
    vx = np.asarray(lpos.data['vx']); vy = np.asarray(lpos.data['vy'])
    speed = np.hypot(vx, vy)

    armed_i = np.where(armed)[0]
    print(f'  duration {t_status[-1]:.1f} s; armed samples {len(armed_i)}')
    if len(armed_i):
        print(f'  armed {t_status[armed_i[0]]:.1f}s -> {t_status[armed_i[-1]]:.1f}s')
        alt_armed = interp(t_status[armed_i], t_lpos, alt)
        spd_armed = interp(t_status[armed_i], t_lpos, speed)
        print(f'  armed: alt max {np.nanmax(alt_armed):.2f} m mean {np.nanmean(alt_armed):.2f} m; '
              f'speed max {np.nanmax(spd_armed):.2f} m/s')

    t_adrc = tsec(adrc)
    for ax, nm in [(0, 'roll'), (1, 'pitch'), (2, 'yaw')]:
        z1 = np.asarray(adrc.data[f'z1[{ax}]'])
        z3 = np.asarray(adrc.data[f'z3[{ax}]'])
        u_ = np.asarray(adrc.data[f'u[{ax}]'])
        e1 = np.asarray(adrc.data[f'e1[{ax}]'])
        print(f'  ADRC[{nm}] all: |z1|max {np.max(np.abs(z1)):.3f}  z3 mean {np.mean(z3):+.2f} '
              f'z3 std {np.std(z3):.2f} |z3|max {np.max(np.abs(z3)):.2f}  '
              f'|u|max {np.max(np.abs(u_)):.3f}  |e1|max {np.max(np.abs(e1)):.3f}')

    if 'z3raw[0]' in adrc.data:
        for ax, nm in [(0, 'roll'), (1, 'pitch'), (2, 'yaw')]:
            z3r = np.asarray(adrc.data[f'z3raw[{ax}]'])
            z3 = np.asarray(adrc.data[f'z3[{ax}]'])
            hit = np.sum(np.abs(z3r - z3) > 1e-6)
            print(f'  z3 auto-sat hits [{nm}]: {hit}/{len(z3)} ({100.0*hit/len(z3):.1f}%) '
                  f'|z3raw|max {np.max(np.abs(z3r)):.1f}')

    t_vad = tsec(vad)
    rr = np.asarray(vad.data['xyz[0]']); pr = np.asarray(vad.data['xyz[1]']); yr = np.asarray(vad.data['xyz[2]'])
    armed_v = interp(t_vad, t_status, armed.astype(float)) > 0.5
    landed_v = interp(t_vad, t_land, landed.astype(float)) > 0.5
    alt_v = interp(t_vad, t_lpos, alt)
    fly = armed_v & (alt_v > 0.4) & (~landed_v)
    n_fly = int(np.sum(fly))
    print(f'  flying samples (armed & alt>0.4m & !landed): {n_fly}')
    if n_fly > 200:
        t0, t1 = t_vad[fly][0], t_vad[fly][-1]
        print(f'  flight window {t0:.1f} - {t1:.1f} s')
        seg = fly
        t_s = t_vad[seg]
        for nm, x in [('roll', rr[seg]), ('pitch', pr[seg]), ('yaw', yr[seg])]:
            print(f'    {nm} rate: RMS {np.degrees(np.std(x)):.2f} deg/s, |max| {np.degrees(np.max(np.abs(x))):.1f} deg/s')
        for nm, x in [('roll', rr[seg]), ('pitch', pr[seg]), ('yaw', yr[seg])]:
            f, a = fft_peak(t_s, x)
            print(f'    {nm} rate FFT dominant: {f:.2f} Hz (amp {a:.4f} rad/s)')
        t_tq = tsec(vtq)
        tq = np.stack([np.asarray(vtq.data[f'xyz[{i}]']) for i in range(3)], axis=1)
        m = (t_tq >= t0) & (t_tq <= t1)
        if np.sum(m) > 10:
            tqs = tq[m]
            for i, nm in enumerate(['roll', 'pitch', 'yaw']):
                print(f'    torque[{nm}]: RMS {np.std(tqs[:, i]):.3f}, |max| {np.max(np.abs(tqs[:, i])):.3f}')
        m2 = (t_adrc >= t0) & (t_adrc <= t1)
        if np.sum(m2) > 10:
            for ax, nm in [(0, 'roll'), (1, 'pitch'), (2, 'yaw')]:
                u_ = np.asarray(adrc.data[f'u[{ax}]'])[m2]
                e1 = np.asarray(adrc.data[f'e1[{ax}]'])[m2]
                z3 = np.asarray(adrc.data[f'z3[{ax}]'])[m2]
                f_u, a_u = fft_peak(t_adrc[m2], u_)
                print(f'    adrc u[{nm}]: RMS {np.std(u_):.4f}, |max| {np.max(np.abs(u_)):.3f}, '
                      f'FFT {f_u:.2f}Hz/{a_u:.4f}; e1 RMS {np.std(e1):.3f}; z3 std {np.std(z3):.2f}')
        if np.sum(m2) > 100:
            for ax, nm in [(0, 'roll'), (1, 'pitch')]:
                u_ = np.asarray(adrc.data[f'u[{ax}]'])[m2]
                e1 = np.asarray(adrc.data[f'e1[{ax}]'])[m2]
                g = np.abs(u_) / (np.abs(e1) + 1e-3)
                print(f'    effective |u|/|e1| [{nm}]: median {np.median(g):.4f}')
    else:
        # no real flight: takeoff region = armed
        print('  NO real flight segment (did not get >0.4m). Analyzing takeoff window:')
        if len(armed_i):
            t_arm = t_status[armed_i[0]]
            m = (t_vad >= t_arm) & (t_vad <= t_arm + 15)
            m2 = (t_adrc >= t_arm) & (t_adrc <= t_arm + 15)
            if np.sum(m) > 100:
                for nm, x in [('roll', rr[m]), ('pitch', pr[m]), ('yaw', yr[m])]:
                    print(f'    takeoff(15s) {nm} rate RMS {np.degrees(np.std(x)):.2f} deg/s |max| {np.degrees(np.max(np.abs(x))):.1f}')
                for nm, x in [('roll', rr[m]), ('pitch', pr[m])]:
                    f, a = fft_peak(t_vad[m], x)
                    print(f'    takeoff {nm} rate FFT dominant: {f:.2f} Hz (amp {a:.4f})')
            if np.sum(m2) > 100:
                for ax, nm in [(0, 'roll'), (1, 'pitch')]:
                    u_ = np.asarray(adrc.data[f'u[{ax}]'])[m2]
                    z3 = np.asarray(adrc.data[f'z3[{ax}]'])[m2]
                    e1 = np.asarray(adrc.data[f'e1[{ax}]'])[m2]
                    f_u, a_u = fft_peak(t_adrc[m2], u_)
                    print(f'    takeoff adrc[{nm}]: |u|max {np.max(np.abs(u_)):.3f} (u RMS {np.std(u_):.4f}), '
                          f'z3 std {np.std(z3):.2f} |z3|max {np.max(np.abs(z3)):.2f}, |e1|max {np.max(np.abs(e1)):.3f}, '
                          f'u FFT {f_u:.2f}Hz/{a_u:.4f}')

    if 'y[0]' in adrc.data:
        y = np.asarray(adrc.data['y[0]'])
        z1 = np.asarray(adrc.data['z1[0]'])
        raw = interp(t_adrc, t_vad, rr)
        ueso = np.asarray(adrc.data['ueso[0]'])
        u_ = np.asarray(adrc.data['u[0]'])
        armed_a = interp(t_adrc, t_status, armed.astype(float)) > 0.5
        if np.sum(armed_a) > 100:
            print(f'  FLT check (roll, armed): raw-y RMS {np.std(raw[armed_a]-y[armed_a]):.4f} rad/s, '
                  f'y-z1 RMS {np.std(y[armed_a]-z1[armed_a]):.4f}')
            print(f'  TAU check (roll, armed): ueso-u RMS {np.std(ueso[armed_a]-u_[armed_a]):.4f}, '
                  f'|ueso-u|max {np.max(np.abs(ueso[armed_a]-u_[armed_a])):.4f}')

    if bat:
        v = np.asarray(bat.data['voltage_v'])
        print(f'  battery: V {v[0]:.1f} -> {v[-1]:.1f} V (min {np.min(v):.1f})')

    if len(armed_i):
        navs, cnt = np.unique(nav[armed_i], return_counts=True)
        print(f'  nav_state while armed: {dict(zip(navs.tolist(), cnt.tolist()))}')
    print()

for n, f in LOGS:
    analyze(n, f)
