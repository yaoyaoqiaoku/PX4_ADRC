/****************************************************************************
 *
 *   Copyright (c) 2026 my_px4_project. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file adrc_params.c
 *
 * Parameters for the ADRC rate controller.
 *
 * === QUICK-START TUNING GUIDE (LADRC mode, ADRC_ESO_MODE=1) ===
 *
 * Per-axis CORE parameters (start here):
 *   1. ADRC_*_B0   — plant gain (calibrate first, see Roll B0 description)
 *   2. ADRC_*_ESO_W — observer bandwidth (3-10x CW)
 *   3. ADRC_*_CW    — controller bandwidth (start 10-15)
 *
 * Real-aircraft hardening (add when needed):
 *   4. ADRC_*_GAMMA — disturbance compensation ratio (0.5-0.7 if oscillating)
 *   5. ADRC_*_FLT   — feedback low-pass (80-150 Hz if gyro noise is an issue)
 *   6. ADRC_*_TAU   — actuator lag (0.01-0.03 for 14" props)
 *
 * Advanced / nonlinear ESO parameters ([ADVANCED] tag):
 *   B01/B02/B03, NB1/NB2, A1/A2, R, DELTA
 *   — only needed when switching to classic ADRC (ESO_MODE=0).
 *   In LADRC mode these are ignored.
 */

/**
 * ADRC NLSEF control law
 *
 * 0: nonlinear fal-based law (Han ADRC, default)
 * 2: linear law (PD-like, recommended for initial tuning)
 *
 * @min 0
 * @max 2
 * @decimal 0
 * @group ADRC Rate Control
 */
PARAM_DEFINE_INT32(ADRC_CTRL_LAW, 2);

/**
 * ADRC ESO structure
 *
 * 0: nonlinear fal-based ESO (classic ADRC)
 * 1: linear ESO (LADRC). beta01 = 2*ESO_W, beta02 = ESO_W^2,
 *    control u0 = CTRL_W * (rate setpoint - z1). Recommended for the
 *    angular rate loop; only two bandwidth knobs to tune. Default.
 *
 * @min 0
 * @max 1
 * @decimal 0
 * @group ADRC Rate Control
 */
PARAM_DEFINE_INT32(ADRC_ESO_MODE, 1);

/**
 * Roll LADRC observer bandwidth w_o [rad/s]
 *
 * [CORE] Primary tuning knob in LADRC mode. Should be roughly 3-10x
 * the controller bandwidth ADRC_ROLL_CW. Higher = faster disturbance
 * estimation but more noise sensitivity.
 *
 * @min 1.0
 * @max 200.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_ESO_W, 50.0f);

/**
 * Roll LADRC controller bandwidth w_c [rad/s]
 *
 * [CORE] Rate loop bandwidth. Start around 10-15 and increase while
 * observing tracking and noise.
 *
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 0.5
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_CW, 11.0f);

/**
 * Roll feedback low-pass cutoff
 *
 * 1st-order low-pass applied to the measured rate before the ESO.
 * Reduces gyro-noise coupling into the observer. 0 disables.
 *
 * @unit Hz
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_FLT, 0.0f);

/**
 * Roll feedback notch center frequency
 *
 * Biquad notch on the measured rate before the ESO. Use to remove a
 * measured airframe/prop resonance that the loop excites (find it in the
 * rate FFT of a hover log). 0 disables.
 *
 * @unit Hz
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_NF, 0.0f);

/**
 * Roll feedback notch bandwidth
 *
 * @unit Hz
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_NBW, 1.0f);

/**
 * Roll actuator-lag time constant
 *
 * Actuator (ESC+prop) lag model. In LADRC mode (ADRC_ESO_MODE=1), when the
 * observer is fast enough relative to the lag (3*ESO_W*TAU > 1) this engages
 * a 3rd-order augmented ESO that estimates the actuator torque as an
 * observer state, so the lag is no longer misread as disturbance (the
 * mechanism behind the low-frequency limit cycle on real airframes). Below
 * that ratio the input is shaped with a 1st-order lag filter instead.
 * Typical 14in prop: 0.01-0.03 s. 0 disables.
 *
 * @unit s
 * @min 0.0
 * @max 0.2
 * @decimal 3
 * @increment 0.001
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_TAU, 0.0f);

/**
 * Roll max ESO disturbance estimate z3
 *
 * Saturation on the total-disturbance estimate [rad/s^2].
 * 0 = auto limit 2*|b0| (recommended safety default).
 *
 * @unit rad/s^2
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_Z3MAX, 0.0f);

/**
 * Roll rate-setpoint feed-forward gain
 *
 * Adds FF * rate_setpoint to the LADRC control law. Improves stick
 * responsiveness (跟手) without raising w_c. 0 disables.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 3
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_FF, 0.0f);

/**
 * Roll LADRC integral gain
 *
 * Integral on the rate error with anti-windup. Helps remove residual offset
 * caused by b0 mismatch and trim. 0 disables (z3 already handles constant
 * disturbances in theory).
 *
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_KI, 0.0f);

/**
 * Roll LADRC integrator output limit ratio
 *
 * Maximum integrator contribution as a fraction of the output range:
 * max |integral| = ILIM / KI. Lower values reduce integrator windup risk
 * at the cost of slower steady-state error removal. 0 disables the limit.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_ILIM, 0.5f);

/**
 * Roll output slew-rate limit
 *
 * Maximum normalized-torque change per second. Smooths step commands and
 * avoids exciting flexible modes. 0 disables.
 *
 * @unit 1/s
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_RAMP, 0.0f);

/**
 * Roll disturbance compensation gain gamma
 *
 * Control law: u = (u0 - gamma*z3) / b0.
 * gamma = 1 fully compensates the estimated total disturbance (classic ADRC);
 * gamma < 1 leaves part of it uncompensated, which dramatically improves
 * robustness against measurement noise and actuator delay (FMT / Jiachi Zou
 * thesis). For a real airframe that oscillates, start at 0.5-0.7 instead of
 * raising the notch or cutting ESO_W further.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_GAMMA, 1.0f);

/**
 * Roll rate-setpoint smoothing cutoff
 *
 * 1st-order low-pass on the rate setpoint ("poor man's TD"): arranges a smooth
 * transition for stick steps so aggressive maneuvers do not excite airframe
 * flexible modes. The smoothed reference is also used by the feed-forward.
 * 0 disables (raw setpoint). Typical 5-15 Hz when stick bang excites wobble.
 *
 * @unit Hz
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.5
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_SPS, 0.0f);

/**
 * Roll disturbance-estimate leaky integration rate
 *
 * @unit 1/s
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_LZ3, 0.0f);

/**
 * Roll adaptive disturbance-estimate filter threshold
 *
 * @unit rad/s^2
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_AF, 0.0f);

/**
 * Pitch LADRC observer bandwidth w_o [rad/s]
 *
 * [CORE] See ADRC_ROLL_ESO_W for tuning guidance.
 *
 * @min 1.0
 * @max 200.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_ESO_W, 50.0f);

/**
 * Pitch LADRC controller bandwidth w_c [rad/s]
 *
 * [CORE] See ADRC_ROLL_CW for tuning guidance.
 *
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 0.5
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_CW, 11.0f);

/**
 * Pitch feedback low-pass cutoff
 *
 * @unit Hz
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_FLT, 0.0f);

/**
 * Pitch feedback notch center frequency
 *
 * @unit Hz
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_NF, 0.0f);

/**
 * Pitch feedback notch bandwidth
 *
 * @unit Hz
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_NBW, 1.0f);

/**
 * Pitch actuator-lag time constant
 *
 * Actuator (ESC+prop) lag model. In LADRC mode, when 3*ESO_W*TAU > 1 this
 * engages a 3rd-order augmented ESO (actuator torque as observer state);
 * below that ratio the input is shaped with a 1st-order lag filter instead.
 * Typical 14in prop: 0.01-0.03 s. 0 disables.
 *
 * @unit s
 * @min 0.0
 * @max 0.2
 * @decimal 3
 * @increment 0.001
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_TAU, 0.0f);

/**
 * Pitch max ESO disturbance estimate z3
 *
 * @unit rad/s^2
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_Z3MAX, 0.0f);

/**
 * Pitch rate-setpoint feed-forward gain
 *
 * @min 0.0
 * @max 2.0
 * @decimal 3
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_FF, 0.0f);

/**
 * Pitch LADRC integral gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_KI, 0.0f);

/**
 * Pitch LADRC integrator output limit ratio
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_ILIM, 0.5f);

/**
 * Pitch output slew-rate limit
 *
 * @unit 1/s
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_RAMP, 0.0f);

/**
 * Pitch disturbance compensation gain gamma
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_GAMMA, 1.0f);

/**
 * Pitch rate-setpoint smoothing cutoff
 *
 * @unit Hz
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.5
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_SPS, 0.0f);

/**
 * Pitch disturbance-estimate leaky integration rate
 *
 * @unit 1/s
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_LZ3, 0.0f);

/**
 * Pitch adaptive disturbance-estimate filter threshold
 *
 * @unit rad/s^2
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_AF, 0.0f);

/**
 * Yaw LADRC observer bandwidth w_o [rad/s]
 *
 * [CORE] See ADRC_ROLL_ESO_W for tuning guidance.
 *
 * @min 1.0
 * @max 200.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_ESO_W, 45.0f);

/**
 * Yaw LADRC controller bandwidth w_c [rad/s]
 *
 * [CORE] See ADRC_ROLL_CW for tuning guidance.
 *
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 0.5
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_CW, 8.0f);

/**
 * Yaw feedback low-pass cutoff
 *
 * @unit Hz
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_FLT, 0.0f);

/**
 * Yaw feedback notch center frequency
 *
 * @unit Hz
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_NF, 0.0f);

/**
 * Yaw feedback notch bandwidth
 *
 * @unit Hz
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_NBW, 1.0f);

/**
 * Yaw actuator-lag time constant
 *
 * Actuator (ESC+prop) lag model. In LADRC mode, when 3*ESO_W*TAU > 1 this
 * engages a 3rd-order augmented ESO (actuator torque as observer state);
 * below that ratio the input is shaped with a 1st-order lag filter instead.
 * Typical 14in prop: 0.01-0.03 s. 0 disables.
 *
 * @unit s
 * @min 0.0
 * @max 0.2
 * @decimal 3
 * @increment 0.001
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_TAU, 0.0f);

/**
 * Yaw max ESO disturbance estimate z3
 *
 * @unit rad/s^2
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_Z3MAX, 0.0f);

/**
 * Yaw rate-setpoint feed-forward gain
 *
 * @min 0.0
 * @max 2.0
 * @decimal 3
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_FF, 0.0f);

/**
 * Yaw LADRC integral gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_KI, 0.0f);

/**
 * Yaw LADRC integrator output limit ratio
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_ILIM, 0.5f);

/**
 * Yaw output slew-rate limit
 *
 * @unit 1/s
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_RAMP, 0.0f);

/**
 * Yaw disturbance compensation gain gamma
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_GAMMA, 1.0f);

/**
 * Yaw rate-setpoint smoothing cutoff
 *
 * @unit Hz
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.5
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_SPS, 0.0f);

/**
 * Yaw disturbance-estimate leaky integration rate
 *
 * @unit 1/s
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_LZ3, 0.0f);

/**
 * Yaw adaptive disturbance-estimate filter threshold
 *
 * @unit rad/s^2
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_AF, 0.0f);

/**
 * Roll ADRC TD fast tracking factor
 *
 * [ADVANCED] Classic ADRC (ESO_MODE=0) only. Unused in LADRC mode.
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_R, 100.0f);

/**
 * Roll ADRC plant control gain b0
 *
 * [CORE] MUST be calibrated: b0 = angular_acceleration / normalized_torque.
 *
 * @min 0.0001
 * @decimal 3
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_B0, 100.0f);

/**
 * Roll ADRC fal() linear interval width
 *
 * [ADVANCED] Nonlinear ESO/NLSEF only.
 *
 * @min 0.0001
 * @decimal 4
 * @increment 0.001
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_DELTA, 0.015f);

/**
 * Roll ADRC ESO gain beta01
 *
 * [ADVANCED] Nonlinear ESO (ADRC_ESO_MODE=0) only. These gains place the observer
 * poles and are coupled to the control-loop sample rate. The defaults
 * (150/250/550) are tuned for ~400 Hz (2.5 ms dt). If your board runs
 * the rate loop at a different frequency, re-tune: faster loops need
 * higher gains, slower loops need lower. In LADRC mode (ESO_MODE=1)
 * these are ignored — use ESO_W instead.
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_B01, 150.0f);

/**
 * Roll ADRC ESO gain beta02
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_B02, 250.0f);

/**
 * Roll ADRC ESO gain beta03
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_B03, 550.0f);

/**
 * Roll ADRC NLSEF gain on e1
 *
 * [ADVANCED] Nonlinear control law (ADRC_CTRL_LAW=0) only.
 *
 * @min 0.0
 * @decimal 1
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_NB1, 10.0f);

/**
 * Roll ADRC NLSEF gain on e2
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0001
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_NB2, 0.0009f);

/**
 * Roll ADRC NLSEF exponent alpha1 (0 < alpha1 < 1)
 *
 * @min 0.1
 * @max 0.99
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_A1, 0.9f);

/**
 * Roll ADRC NLSEF exponent alpha2 (alpha2 > 1)
 *
 * @min 1.01
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_A2, 1.5f);

/**
 * Roll ADRC control output upper limit
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_UMX, 1.0f);

/**
 * Roll ADRC control output lower limit
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_ROLL_UMN, -1.0f);

/**
 * Pitch ADRC TD fast tracking factor
 *
 * [ADVANCED] Classic ADRC (ESO_MODE=0) only. Unused in LADRC mode.
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_R, 100.0f);

/**
 * Pitch ADRC plant control gain b0
 *
 * [CORE] Must be calibrated. See ADRC_ROLL_B0 for procedure.
 *
 * @min 0.0001
 * @decimal 3
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_B0, 100.0f);

/**
 * Pitch ADRC fal() linear interval width
 *
 * [ADVANCED] Nonlinear ESO/NLSEF only.
 *
 * @min 0.0001
 * @decimal 4
 * @increment 0.001
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_DELTA, 0.015f);

/**
 * Pitch ADRC ESO gain beta01
 *
 * [ADVANCED] Nonlinear ESO (ADRC_ESO_MODE=0) only. Sample-rate dependent — see
 * ADRC_ROLL_B01 for the full explanation. Ignored in LADRC mode.
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_B01, 150.0f);

/**
 * Pitch ADRC ESO gain beta02
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_B02, 250.0f);

/**
 * Pitch ADRC ESO gain beta03
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_B03, 550.0f);

/**
 * Pitch ADRC NLSEF gain on e1
 *
 * [ADVANCED] Nonlinear control law (ADRC_CTRL_LAW=0) only.
 *
 * @min 0.0
 * @decimal 1
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_NB1, 10.0f);

/**
 * Pitch ADRC NLSEF gain on e2
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0001
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_NB2, 0.0009f);

/**
 * Pitch ADRC NLSEF exponent alpha1 (0 < alpha1 < 1)
 *
 * @min 0.1
 * @max 0.99
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_A1, 0.9f);

/**
 * Pitch ADRC NLSEF exponent alpha2 (alpha2 > 1)
 *
 * @min 1.01
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_A2, 1.5f);

/**
 * Pitch ADRC control output upper limit
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_UMX, 1.0f);

/**
 * Pitch ADRC control output lower limit
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_PITCH_UMN, -1.0f);

/**
 * Yaw ADRC TD fast tracking factor
 *
 * [ADVANCED] Classic ADRC (ESO_MODE=0) only. Unused in LADRC mode.
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_R, 50.0f);

/**
 * Yaw ADRC plant control gain b0
 *
 * [CORE] Must be calibrated. See ADRC_ROLL_B0 for procedure.
 *
 * @min 0.0001
 * @decimal 3
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_B0, 20.0f);

/**
 * Yaw ADRC fal() linear interval width
 *
 * [ADVANCED] Nonlinear ESO/NLSEF only.
 *
 * @min 0.0001
 * @decimal 4
 * @increment 0.001
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_DELTA, 0.015f);

/**
 * Yaw ADRC ESO gain beta01
 *
 * [ADVANCED] Nonlinear ESO (ADRC_ESO_MODE=0) only. Sample-rate dependent — see
 * ADRC_ROLL_B01 for the full explanation. Ignored in LADRC mode.
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_B01, 80.0f);

/**
 * Yaw ADRC ESO gain beta02
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_B02, 120.0f);

/**
 * Yaw ADRC ESO gain beta03
 *
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_B03, 200.0f);

/**
 * Yaw ADRC NLSEF gain on e1
 *
 * [ADVANCED] Nonlinear control law (ADRC_CTRL_LAW=0) only.
 *
 * @min 0.0
 * @decimal 1
 * @increment 0.1
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_NB1, 5.0f);

/**
 * Yaw ADRC NLSEF gain on e2
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0001
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_NB2, 0.0004f);

/**
 * Yaw ADRC NLSEF exponent alpha1 (0 < alpha1 < 1)
 *
 * @min 0.1
 * @max 0.99
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_A1, 0.9f);

/**
 * Yaw ADRC NLSEF exponent alpha2 (alpha2 > 1)
 *
 * @min 1.01
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_A2, 1.5f);

/**
 * Yaw ADRC control output upper limit
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_UMX, 1.0f);

/**
 * Yaw ADRC control output lower limit
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group ADRC Rate Control
 */
PARAM_DEFINE_FLOAT(ADRC_YAW_UMN, -1.0f);

/**
 * Acro mode maximum roll rate
 *
 * Full stick deflection leads to this rate.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_R_MAX, 100.f);

/**
 * Acro mode maximum pitch rate
 *
 * Full stick deflection leads to this rate.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_P_MAX, 100.f);

/**
 * Acro mode maximum yaw rate
 *
 * Full stick deflection leads to this rate.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_Y_MAX, 100.f);

/**
 * Acro mode roll, pitch expo factor
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_EXPO, 0.f);

/**
 * Acro mode yaw expo factor
 *
 * Exponential factor for tuning the input curve shape.
 *
 * 0 Purely linear input curve
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_EXPO_Y, 0.f);

/**
 * Acro mode roll, pitch super expo factor
 *
 * "Superexponential" factor for refining the input curve shape tuned using MC_ACRO_EXPO.
 *
 * 0 Pure Expo function
 * 0.7 reasonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_SUPEXPO, 0.f);

/**
 * Acro mode yaw super expo factor
 *
 * "Superexponential" factor for refining the input curve shape tuned using MC_ACRO_EXPO_Y.
 *
 * 0 Pure Expo function
 * 0.7 reasonable shape enhancement for intuitive stick feel
 * 0.95 very strong bent input curve only near maxima have effect
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Multicopter Acro Mode
 */
PARAM_DEFINE_FLOAT(MC_ACRO_SUPEXPOY, 0.f);

/**
 * Battery scaling enabled
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);
