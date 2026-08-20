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
 * @file Adrc.hpp
 *
 * Single-axis ADRC (Active Disturbance Rejection Control) implementation.
 *
 * Ported from zhaohaojie1998/Control-Algorithm (ctrl_cpp/adrc), adapted for
 * PX4 real-time use:
 *  - float precision (was double)
 *  - dynamic dt handling in the caller
 *  - standard disturbance compensation u = (u0 - z3) / b0
 *    (the upstream code computes u0 - z3/b0, which is inconsistent with the
 *     ESO formulation; fixed here, see design doc section 5)
 *  - the control applied to the plant (clamped) is fed back into the ESO,
 *    which provides natural anti-windup
 *
 * Real-aircraft hardening (2026-08):
 *  - optional 1st-order low-pass on the measured rate fed to the ESO
 *  - optional biquad notch on the measured rate (airframe/prop resonance)
 *  - optional 1st-order actuator-lag model shaping the ESO input (b0*u_eso):
 *    prevents the ESO from misreading actuator delay as disturbance and
 *    exciting a low-frequency limit cycle
 *  - ESO disturbance estimate saturation (z3_max, default auto = 2*|b0|)
 *  - LADRC control law extended with setpoint feed-forward and a bounded,
 *    anti-windup integral (removes residual offset with b0 mismatch)
 *  - optional slew-rate limit on the normalized torque output
 *  - partial disturbance compensation gain gamma (FMT / Jiachi Zou thesis):
 *    u = (u0 - gamma*z3)/b0, gamma in [0,1] trades disturbance rejection for
 *    robustness against measurement noise and actuator delay
 *  - optional 1st-order low-pass on the rate setpoint ("poor man's TD"):
 *    arranges a smooth transition for stick steps, reducing excitation of
 *    flexible airframe modes during aggressive maneuvers
 */

#pragma once

class Adrc
{
public:
	Adrc() = default;

	/**
	 * @param r            TD fast tracking factor
	 * @param b0           plant control gain (must be calibrated)
	 * @param delta        fal() linear interval width
	 * @param eso_beta01   ESO gain 1
	 * @param eso_beta02   ESO gain 2
	 * @param eso_beta03   ESO gain 3
	 * @param nlsef_beta1  NLSEF gain on e1
	 * @param nlsef_beta2  NLSEF gain on e2
	 * @param nlsef_alpha1 0 < alpha1 < 1
	 * @param nlsef_alpha2 alpha2 > 1
	 * @param u_min        control output lower limit
	 * @param u_max        control output upper limit
	 */
	void setParameters(float r, float b0, float delta,
			   float eso_beta01, float eso_beta02, float eso_beta03,
			   float nlsef_beta1, float nlsef_beta2,
			   float nlsef_alpha1, float nlsef_alpha2,
			   float u_min, float u_max);

	/** Select the NLSEF control law: 0 = nonlinear fal, 2 = linear (PD-like). */
	void setControlLaw(int law);

	/** Select the ESO structure:
	 *  0 = nonlinear fal ESO (classic ADRC, default)
	 *  1 = linear ESO (LADRC): beta01 = 2*w_o, beta02 = w_o^2, control u0 = w_c*e1 */
	void setEsoMode(int mode);

	/** Observer bandwidth w_o [rad/s] for LADRC mode. */
	void setObserverBandwidth(float w);

	/** Controller bandwidth w_c [rad/s] for LADRC mode. */
	void setControllerBandwidth(float w);

	/** Actuator lag time constant tau [s] shaping the ESO input (0 = none). */
	void setActuatorTau(float tau);

	/** Feedback low-pass cutoff [Hz] applied before the ESO (0 = none). */
	void setFeedbackFilter(float hz);

	/** Biquad notch on the feedback: center freq [Hz], bandwidth [Hz] (freq <= 0 disables). */
	void setNotch(float freq_hz, float bw_hz);

	/** Max |z3| [rad/s^2]; <= 0 uses auto limit 2*|b0|. */
	void setZ3Max(float z3_max);

	/** Rate-setpoint feed-forward gain (LADRC only, 0 = none). */
	void setFeedforward(float ff);

	/** Integral gain on the rate error (LADRC only, 0 = none). */
	void setIntegralGain(float ki);

	/** Integral limit ratio: max |integral| = ratio / ki (0 disables). */
	void setIntegratorLimit(float ratio);

	/** Output slew-rate limit [normalized torque / s] (0 = none). */
	void setOutputRateLimit(float rate_max);

	/** Disturbance compensation gain gamma (0..2, default 1 = full compensation). */
	void setGamma(float gamma);

	/** Rate-setpoint low-pass cutoff [Hz] (0 = none). */
	void setSetpointFilter(float hz);

	/** Torque actually achieved by the control allocator for this axis
	 *  (command minus unallocated). Overrides the ESO input for this cycle so
	 *  actuator saturation is not estimated as disturbance (anti-windup).
	 *  Must be called before update(). */
	void setAppliedTorque(float u_applied);

	/** Control-allocator saturation status for integral anti-windup
	 *  (mirrors the stock PID rate controller). */
	void setSaturationStatus(bool saturated_positive, bool saturated_negative);

	/** Run one control step. Returns the clamped control output. */
	float update(float setpoint, float feedback, float dt);

	/** Reset all internal states; seed the ESO position estimate with feedback. */
	void reset(float feedback = 0.0f);

	/** Zero only the disturbance estimate (ESO z3), keep TD and velocity states.
	 *  Mirrors the stock PID integrator reset while landed. */
	void resetIntegral();

	/* debug accessors — semantics depend on ESO mode:
	 *   nonlinear ESO (eso_mode=0): z1=rate, z2=acceleration, z3=disturbance
	 *   linear 2-state ESO (eso_mode=1, tau=0): z1=rate, z2=0, z3=disturbance
	 *   augmented 3rd-order ESO (eso_mode=1, tau>0): z1=rate, z2=actuator_torque, z3=disturbance */
	float getV1() const { return _v1; }
	float getV2() const { return _v2; }
	float getZ1() const { return _z1; }
	float getZ2() const { return _z2; }
	float getZ3() const { return _z3; }
	float getE1() const { return _e1; }
	float getE2() const { return _e2; }
	float getU() const { return _u; }
	float getY() const { return _y; }
	float getUEso() const { return _u_eso; }
	float getIntegral() const { return _integ; }
	float getZ3Raw() const { return _z3_raw; }
	float getSp() const { return _sp_used; }

private:
	/* 1. tracking differentiator */
	void updateTd(float v, float h);

	/* 2. extended state observer */
	void updateEso(float y, float h);

	/* 3. nonlinear state error feedback */
	float updateNlsef();

	/* linear 2-state ESO (LADRC): z1 = rate, z3 = total disturbance */
	void updateEsoLinear(float y, float h);

	/* 3rd-order augmented ESO (LADRC): z1 = rate, z2 = actuator torque state,
	 * z3 = total disturbance. Replaces the input-lag shaping when the
	 * pole-placement gains are positive (see augmentedEsoEnabled()). */
	void updateEsoLinearAugmented(float y, float h);

	/* true when the augmented ESO is usable: tau > 0 and both pole-placement
	 * gains are positive (beta1 > 0 and beta2 > 0) */
	bool augmentedEsoEnabled() const;

	/* actuator-lag shaped ESO input */
	float getEsoInput(float h);

	/* saturate the disturbance estimate */
	void saturateZ3();

	/* output clamping + slew-rate limiting */
	float applyLimits(float u_new, float dt);

	/* biquad notch support */
	void updateNotch(float dt);
	float applyNotch(float x);

	/* nonlinear helpers */
	static float fhan(float x1, float x2, float r, float h);
	static float fal(float e, float alpha, float delta);
	static float fsg(float x, float d);
	static float signf(float x);

	float _r{100.0f};
	float _b0{1.0f};
	float _delta{0.015f};
	float _eso_beta01{150.0f};
	float _eso_beta02{250.0f};
	float _eso_beta03{550.0f};
	float _nlsef_beta1{10.0f};
	float _nlsef_beta2{0.0009f};
	float _nlsef_alpha1{0.9f};
	float _nlsef_alpha2{1.5f};
	float _u_min{-1.0f};
	float _u_max{1.0f};
	int _ctrl_law{0};
	int _eso_mode{0};
	float _eso_w{50.0f};
	float _ctrl_w{15.0f};

	/* real-aircraft hardening */
	float _tau{0.0f};
	float _flt_hz{0.0f};
	float _z3_max{0.0f};
	float _ff{0.0f};
	float _ki{0.0f};
	float _u_rate_max{0.0f};
	float _gamma{1.0f};
	float _sp_flt_hz{0.0f};
	float _ilim_ratio{0.5f};	/* max |integral| = ratio / ki */

	/* internal states */
	float _v1{0.0f};
	float _v2{0.0f};
	float _z1{0.0f};
	float _z2{0.0f};
	float _z3{0.0f};
	float _u{0.0f};
	float _e1{0.0f};
	float _e2{0.0f};
	float _y{0.0f};		/* conditioned feedback fed to the ESO */
	float _y_filt{0.0f};	/* low-pass state */
	float _u_eso{0.0f};	/* actuator-shaped ESO input state (input-lag model);
				 * in augmented mode mirrors z2 for adrc_status.ueso */
	float _u_eso_in{0.0f};	/* torque fed to the ESO: the command, or the achieved
				 * torque when the control allocator saturates (anti-windup) */
	float _u_prev{0.0f};	/* previous output for slew limiting */
	float _integ{0.0f};	/* LADRC integral state */
	float _z3_raw{0.0f};	/* z3 before saturation (debug) */
	float _sp_filt{0.0f};	/* smoothed rate setpoint state */
	float _sp_used{0.0f};	/* setpoint actually fed to the controller (debug) */
	bool _sat_pos{false};	/* control-allocator saturated positive (torque) */
	bool _sat_neg{false};	/* control-allocator saturated negative (torque) */

	/* biquad notch state */
	float _notch_hz{0.0f};
	float _notch_bw{1.0f};
	float _notch_fs{0.0f};
	float _notch_b0{1.0f};
	float _notch_b1{0.0f};
	float _notch_b2{0.0f};
	float _notch_a1{0.0f};
	float _notch_a2{0.0f};
	float _notch_x1{0.0f};
	float _notch_x2{0.0f};
	float _notch_y1{0.0f};
	float _notch_y2{0.0f};
};
