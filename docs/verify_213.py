#!/usr/bin/env python3
"""Verify attitude-loop-driving hypothesis in log_213 and check saturation."""
import numpy as np
from pyulog import ULog

f = '/home/liuyao/下载/rizhi/log_213_2026-8-13-16-34-02.ulg'
u = ULog(f)

def ds(n):
    for d in u.data_list:
        if d.name == n:
            return d
    return None

def tsec(d):
    return (np.asarray(d.data['timestamp']) - d.data['timestamp'][0]) * 1e-6

def interp(tn, to, yo):
    return np.interp(tn, to, yo)

def fft_peak(t, x, fmin=0.3, fmax=10.0):
    t = t - t[0]
    dt = np.median(np.diff(t))
    x = x - np.mean(x)
    n = len(x)
    if n < 64: return np.nan, np.nan
    win = np.hanning(n)
    X = np.fft.rfft(x * win)
    fr = np.fft.rfftfreq(n, dt)
    mag = 2.0 * np.abs(X) / np.sum(win)
    m = (fr >= fmin) & (fr <= fmax)
    if not np.any(m): return np.nan, np.nan
    i = np.argmax(mag[m])
    return fr[m][i], mag[m][i]

vad = ds('vehicle_angular_velocity')
vrs = ds('vehicle_rates_setpoint')
att = ds('vehicle_attitude')
attsp = ds('vehicle_attitude_setpoint')
amot = ds('actuator_motors')
cas = ds('control_allocator_status')
land = ds('vehicle_land_detected')
lpos = ds('vehicle_local_position')
status = ds('vehicle_status')

t_vad = tsec(vad)
t_vrs = tsec(vrs)
t_att = tsec(att)
t_amot = tsec(amot)
t_lpos = tsec(lpos)
t_status = tsec(status)
t_land = tsec(land)

# flight window: alt>0.4 & armed
alt = -np.asarray(lpos.data['z'])
armed = np.asarray(status.data['arming_state']) == 2
landed = np.asarray(land.data['landed'])
t0, t1 = 6.5, 48.8  # from previous analysis
m = (t_vad >= t0) & (t_vad <= t1)

rr = np.asarray(vad.data['xyz[0]'])
mr = (t_vrs >= t0) & (t_vrs <= t1)
rsp = np.asarray(vrs.data['roll'])
psp = np.asarray(vrs.data['pitch'])
ysp = np.asarray(vrs.data['yaw'])

print('--- rate setpoint (attitude loop output) in flight window ---')
for nm, x in [('roll', rsp[mr]), ('pitch', psp[mr]), ('yaw', ysp[mr])]:
    print(f'  setpoint {nm}: RMS {np.degrees(np.std(x)):.2f} deg/s, |max| {np.degrees(np.max(np.abs(x))):.1f} deg/s')
for nm, x in [('roll', rsp[mr]), ('pitch', psp[mr])]:
    f, a = fft_peak(t_vrs[mr], x)
    print(f'  setpoint {nm} FFT: {f:.2f} Hz amp {np.degrees(a):.2f} deg/s')

# measured rate in same window
for nm, x in [('roll', rr[m]), ('pitch', np.asarray(vad.data['xyz[1]'])[m])]:
    f, a = fft_peak(t_vad[m], x)
    print(f'  measured {nm} FFT: {f:.2f} Hz amp {np.degrees(a):.2f} deg/s')

# attitude error in window
q = np.stack([np.asarray(att.data['q[%d]' % i]) for i in range(4)], axis=1)
qs = np.stack([np.asarray(attsp.data['q_d[%d]' % i]) for i in range(4)], axis=1)
# small-angle roll error from quaternions: use att control's definition approx via z-axis
# simpler: roll/pitch euler
def quat2euler(qq):
    w, x, y, z = qq
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw
roll = np.array([quat2euler(qq)[0] for qq in q])
pitch = np.array([quat2euler(qq)[1] for qq in q])
rollsp = np.array([quat2euler(qq)[0] for qq in qs])
pitchsp = np.array([quat2euler(qq)[1] for qq in qs])
matt = (t_att >= t0) & (t_att <= t1)
print('--- attitude vs setpoint (deg) in flight window ---')
for nm, a, asp in [('roll', roll, rollsp), ('pitch', pitch, pitchsp)]:
    err = np.degrees(a[matt] - asp[matt])
    print(f'  att err {nm}: RMS {np.std(err):.2f} deg, |max| {np.max(np.abs(err)):.2f} deg')
    f, aa = fft_peak(t_att[matt], err)
    print(f'  att err {nm} FFT: {f:.2f} Hz amp {aa:.2f} deg')

# motors & allocator saturation
m2 = (t_amot >= t0) & (t_amot <= t1)
if np.sum(m2) > 10:
    mot = np.stack([np.asarray(amot.data[f'control[{i}]']) for i in range(4)], axis=1)[m2]
    print('--- actuator_motors (normalized 0..1) ---')
    for i in range(4):
        print(f'  motor{i}: min {np.min(mot[:, i]):.3f} max {np.max(mot[:, i]):.3f} mean {np.mean(mot[:, i]):.3f}')

t_cas = tsec(cas)
m3 = (t_cas >= t0) & (t_cas <= t1)
if np.sum(m3) > 10:
    ua = np.stack([np.asarray(cas.data[f'unallocated_torque[{i}]']) for i in range(3)], axis=1)[m3]
    ach = np.asarray(cas.data['torque_setpoint_achieved'])[m3]
    print(f'  allocator torque_achieved: {np.mean(ach):.1%} of samples')
    print(f'  unallocated torque |max| per axis: {np.max(np.abs(ua), axis=0)}')

# ground effect in window
t_l2 = tsec(land)
m4 = (t_l2 >= t0) & (t_l2 <= t1)
ige = np.asarray(land.data['in_ground_effect'])[m4]
gc = np.asarray(land.data['ground_contact'])[m4]
print(f'  in_ground_effect: {np.mean(ige):.0%} of flight window; ground_contact: {np.mean(gc):.0%}')

# altitude profile in window
ma = (t_lpos >= 0)
print('  alt: min %.2f max %.2f mean %.2f' % (np.min(alt[ma]), np.max(alt[ma]), np.mean(alt[ma])))

# vertical velocity
vz = np.asarray(lpos.data['vz'])[ma]
print('  vz: min %.2f max %.2f (m/s, up+)' % (-np.min(vz), -np.max(vz)))
