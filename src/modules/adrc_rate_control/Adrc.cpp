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
 * @file Adrc.cpp
 */

#include "Adrc.hpp"

#include <cmath>
#include <mathlib/math/Limits.hpp>

namespace
{
constexpr float kPiF = 3.14159265358979323846f;
}

void
Adrc::setParameters(float r, float b0, float delta,
		    float eso_beta01, float eso_beta02, float eso_beta03,
		    float nlsef_beta1, float nlsef_beta2,
		    float nlsef_alpha1, float nlsef_alpha2,
		    float u_min, float u_max)
{
	_r = r;
	_b0 = b0;
	_delta = delta;
	_eso_beta01 = eso_beta01;
	_eso_beta02 = eso_beta02;
	_eso_beta03 = eso_beta03;
	_nlsef_beta1 = nlsef_beta1;
	_nlsef_beta2 = nlsef_beta2;
	_nlsef_alpha1 = nlsef_alpha1;
	_nlsef_alpha2 = nlsef_alpha2;
	_u_min = u_min;
	_u_max = u_max;
}

void
Adrc::setControlLaw(int law)
{
	_ctrl_law = law;
}

void
Adrc::setEsoMode(int mode)
{
	_eso_mode = mode;
}

void
Adrc::setObserverBandwidth(float w)
{
	_eso_w = w;
}

void
Adrc::setControllerBandwidth(float w)
{
	_ctrl_w = w;
}

void
Adrc::setActuatorTau(float tau)
{
	_tau = fmaxf(tau, 0.0f);
}

void
Adrc::setFeedbackFilter(float hz)
{
	_flt_hz = fmaxf(hz, 0.0f);
}

void
Adrc::setNotch(float freq_hz, float bw_hz)
{
	_notch_hz = fmaxf(freq_hz, 0.0f);
	_notch_bw = fmaxf(bw_hz, 0.1f);
	_notch_fs = 0.0f;	/* force coefficient recompute on next update */
	_notch_x1 = _notch_x2 = _notch_y1 = _notch_y2 = 0.0f;
}

void
Adrc::setZ3Max(float z3_max)
{
	_z3_max = fmaxf(z3_max, 0.0f);
}

void
Adrc::setFeedforward(float ff)
{
	_ff = fmaxf(ff, 0.0f);
}

void
Adrc::setIntegralGain(float ki)
{
	_ki = fmaxf(ki, 0.0f);
}

void
Adrc::setOutputRateLimit(float rate_max)
{
	_u_rate_max = fmaxf(rate_max, 0.0f);
}

void
Adrc::setIntegratorLimit(float ratio)
{
	_ilim_ratio = fmaxf(ratio, 0.0f);
}

void
Adrc::setGamma(float gamma)
{
	_gamma = math::constrain(gamma, 0.0f, 2.0f);
}

void
Adrc::setSetpointFilter(float hz)
{
	_sp_flt_hz = fmaxf(hz, 0.0f);
}

void
Adrc::setAppliedTorque(float u_applied)
{
	/* Override the ESO input with the torque the control allocator actually
	 * achieved (command minus unallocated). Without this, allocator
	 * saturation would be interpreted by the ESO as a disturbance and
	 * actively compensated, driving z3 (and the output) into windup. */
	_u_eso_in = u_applied;
}

void
Adrc::setSaturationStatus(bool saturated_positive, bool saturated_negative)
{
	_sat_pos = saturated_positive;
	_sat_neg = saturated_negative;
}

float
Adrc::update(float setpoint, float feedback, float dt)
{
	/* optional setpoint smoothing ("poor man's TD"): a 1st-order low-pass on the
	 * rate reference arranges a smooth transition for stick steps, so the loop
	 * does not hammer the airframe's flexible modes. The smoothed reference is
	 * used for the error, the feed-forward and the TD alike. */
	float sp = setpoint;

	if (_sp_flt_hz > 0.0f) {
		const float alpha = dt / (dt + 1.0f / (2.0f * kPiF * _sp_flt_hz));
		_sp_filt += alpha * (sp - _sp_filt);
		sp = _sp_filt;
	}

	_sp_used = sp;

	/* optional feedback conditioning: LPF then notch, fed to the ESO */
	float y = feedback;

	if (_flt_hz > 0.0f) {
		const float alpha = dt / (dt + 1.0f / (2.0f * kPiF * _flt_hz));
		_y_filt += alpha * (y - _y_filt);
		y = _y_filt;
	}

	if (_notch_hz > 0.0f) {
		updateNotch(dt);
		y = applyNotch(y);
	}

	_y = y;

	if (_eso_mode == 1) {
		/* LADRC: linear control + 2-state linear ESO, or 3rd-order augmented
		 * ESO (actuator lag modeled as an observer state) when the actuator
		 * time constant is set and the pole-placement gains are positive
		 * (beta1 > 0 and beta2 > 0, see augmentedEsoEnabled()). */
		if (augmentedEsoEnabled()) {
			updateEsoLinearAugmented(y, dt);

		} else {
			updateEsoLinear(y, dt);
		}

		_e1 = sp - _z1;
		_e2 = 0.0f;
		_v1 = 0.0f;
		_v2 = 0.0f;

		const float u0 = _ctrl_w * _e1 + _ff * sp + _ki * _integ;
		const float u_new = (_b0 > 1e-6f) ? (u0 - _gamma * _z3) / _b0 : u0;

		/* anti-windup integration: never integrate into an output-clamped or
		 * control-allocator-saturated direction (mirrors the stock PID) */
		if (_ki > 0.0f) {
			float e1_int = _e1;

			if ((u_new >= _u_max) || _sat_pos) {
				e1_int = math::min(e1_int, 0.0f);
			}

			if ((u_new <= _u_min) || _sat_neg) {
				e1_int = math::max(e1_int, 0.0f);
			}

			_integ += e1_int * dt;
			const float i_limit = (_ilim_ratio > 0.0f) ? _ilim_ratio / fmaxf(_ki, 1e-6f) : 0.0f;
			_integ = math::constrain(_integ, -i_limit, i_limit);
		}

		_u = applyLimits(u_new, dt);

		/* Only update the ESO input with the controller output when the
		 * control allocator has NOT reported saturation. When saturated,
		 * setAppliedTorque() feeds the achieved torque (anti-windup);
		 * do not overwrite it here. */
		if (!_sat_pos && !_sat_neg) {
			_u_eso_in = _u;
		}

		return _u;
	}

	/* 1. tracking differentiator */
	updateTd(sp, dt);

	/* 2. extended state observer (uses the clamped control applied last cycle) */
	updateEso(y, dt);

	/* 3. nonlinear state error feedback + disturbance compensation */
	const float u0 = updateNlsef();
	const float u_new = (_b0 > FLT_EPSILON) ? (u0 - _gamma * _z3) / _b0 : u0;

	/* clamp, slew-limit and feed the actual control back to the ESO next cycle */
	_u = applyLimits(u_new, dt);

	/* Same anti-windup logic as the LADRC branch: do not overwrite the
	 * achieved torque fed back by the control allocator. */
	if (!_sat_pos && !_sat_neg) {
		_u_eso_in = _u;
	}

	return _u;
}

void
Adrc::reset(float feedback)
{
	_v1 = 0.0f;
	_v2 = 0.0f;
	_z1 = feedback;
	_z2 = 0.0f;
	_z3 = 0.0f;
	_u = 0.0f;
	_u_prev = 0.0f;
	_u_eso = 0.0f;
	_u_eso_in = 0.0f;
	_y_filt = feedback;
	_y = feedback;
	_integ = 0.0f;
	_sp_filt = feedback;
	_sp_used = feedback;
	_z3_raw = 0.0f;
	_sat_pos = false;
	_sat_neg = false;
	_notch_x1 = _notch_x2 = _notch_y1 = _notch_y2 = 0.0f;
	_e1 = 0.0f;
	_e2 = 0.0f;
}

void
Adrc::resetIntegral()
{
	_z3 = 0.0f;
	_integ = 0.0f;
	/* 3rd-order augmented ESO keeps the actuator torque state in z2; a stale
	 * value here would seed the observer with a phantom initial torque on the
	 * next takeoff (landed -> armed transition), so clear it as well. */
	_z2 = 0.0f;
	/* the input-lag ESO input state too: a stale value would bias the v3 path
	 * on the next arming cycle (in augmented mode _u_eso mirrors z2) */
	_u_eso = 0.0f;
}

void
Adrc::updateTd(float v, float h)
{
	const float fh = fhan(_v1 - v, _v2, _r, h);
	_v1 += h * _v2;
	_v2 += h * fh;
}

void
Adrc::updateEso(float y, float h)
{
	const float u_eso = getEsoInput(h);
	const float e = _z1 - y;
	const float fe = fal(e, 0.5f, _delta);
	const float fe1 = fal(e, 0.25f, _delta);

	_z1 += h * (_z2 - _eso_beta01 * e);
	_z2 += h * (_z3 - _eso_beta02 * fe + _b0 * u_eso);
	_z3 += h * (-_eso_beta03 * fe1);

	saturateZ3();
}

void
Adrc::updateEsoLinear(float y, float h)
{
	const float u_eso = getEsoInput(h);

	/* linear 2-state ESO gains are derived from the observer bandwidth only */
	const float beta01 = 2.0f * _eso_w;
	const float beta02 = _eso_w * _eso_w;
	const float e = _z1 - y;

	/* z1 = angular rate estimate, z3 = total disturbance estimate */
	_z1 += h * (_z3 - beta01 * e + _b0 * u_eso);
	_z3 += h * (-beta02 * e);

	saturateZ3();
}

bool
Adrc::augmentedEsoEnabled() const
{
	if (_tau <= 0.0f) {
		return false;
	}

	const float inv_tau = 1.0f / _tau;
	const float w = _eso_w;
	const float w2 = w * w;
	const float beta1 = 3.0f * w - inv_tau;
	const float beta2 = (3.0f * w2 - beta1 * inv_tau - _tau * w2 * w) / fmaxf(_b0, 1e-6f);

	/* usable only while both pole-placement gains stay positive: beta1 > 0
	 * keeps the direct error feedback positive, and beta2 > 0 keeps the
	 * actuator-state correction sign right. Note sign(beta2) = sign(1-w*tau):
	 * beta2 * tau^2 * b0 == (1-w*tau)^3, so the usable band is
	 * 1/3 < w*tau < 1 — outside it the z2 channel is degenerate or inverted
	 * and the conditional rail logic would fight the release from the rail */
	return (beta1 > 0.0f) && (beta2 > 0.0f);
}

void
Adrc::updateEsoLinearAugmented(float y, float h)
{
	/* 3rd-order augmented ESO for the actuator-lag plant:
	 *   tau * x_a_dot = u - x_a        (actuator, x_a = achieved torque)
	 *   rate_dot      = b0 * x_a + f   (f = total disturbance)
	 * states: z1 = rate, z2 = actuator torque state, z3 = total disturbance.
	 * Observer gains place all error poles at -w_o:
	 *   beta1 = 3*w_o - 1/tau
	 *   beta2 = (3*w_o^2 - beta1/tau - tau*w_o^3) / b0
	 *   beta3 = tau * w_o^3
	 * The actuator lag is part of the observer model instead of a pre-filter,
	 * so the observer no longer misreads the lag as disturbance — the
	 * mechanism that excited the 2-3 Hz limit cycle on the real airframe.
	 * Engaged only when the pole-placement gains are positive (beta1 > 0 and
	 * beta2 > 0, see augmentedEsoEnabled()); otherwise the caller falls back
	 * to the 2-state ESO with input-lag shaping.
	 *
	 * The actuator is modeled as a *saturated* first-order element: its output
	 * torque is physically bounded to the rail [-1,1]. The observer models
	 * this directly with conditional integration on the actuator state: while
	 * z2 is pinned at the rail and the driving torque (command + observer
	 * correction) pushes further, hold the state instead of integrating the
	 * excess. A blind clamp-after-integration would accumulate the correction
	 * term while pinned, keeping the estimate stuck at the rail longer than
	 * the physical actuator (robustness, not performance: with the command
	 * pre-clamped and the control-allocator feedback feeding the achieved
	 * torque, the closed-loop effect of this nonlinearity is small). */
	const float e = _z1 - y;

	const float inv_tau = 1.0f / _tau;
	const float w = _eso_w;
	const float w2 = w * w;
	const float w3 = w2 * w;
	const float beta1 = 3.0f * w - inv_tau;
	const float beta2 = (3.0f * w2 - beta1 * inv_tau - _tau * w3) / fmaxf(_b0, 1e-6f);
	const float beta3 = _tau * w3;

	/* actuator state with conditional integration at the torque rail */
	const float z2_dot = (_u_eso_in - _z2) * inv_tau - beta2 * e;

	if ((_z2 >= 1.0f && z2_dot > 0.0f) || (_z2 <= -1.0f && z2_dot < 0.0f)) {
		/* pinned at the rail, being driven further: hold */

	} else {
		_z2 += h * z2_dot;
	}

	_z1 += h * (_b0 * _z2 + _z3 - beta1 * e);
	_z3 += h * (-beta3 * e);

	/* safety clamp (conditional integration above should keep z2 in range) */
	_z2 = math::constrain(_z2, -1.0f, 1.0f);

	/* debug: in augmented mode adrc_status.ueso reports the actuator torque
	 * state estimate (z2) instead of the stale input-lag state */
	_u_eso = _z2;

	saturateZ3();
}

float
Adrc::getEsoInput(float h)
{
	float u_eso = _u_eso_in;

	/* shape the ESO input with a 1st-order actuator-lag model:
	 * the plant sees the torque only after tau, so feed the ESO the lagged
	 * command instead of the instantaneous one */
	if (_tau > 0.0f) {
		const float alpha = h / (h + _tau);
		_u_eso += alpha * (_u_eso_in - _u_eso);
		u_eso = _u_eso;
	}

	return u_eso;
}

void
Adrc::saturateZ3()
{
	_z3_raw = _z3;

	float limit = _z3_max;

	if (limit <= 0.0f) {
		/* auto: the disturbance estimate should never exceed roughly twice the
		 * maximum angular acceleration the axis can physically produce */
		limit = 2.0f * fabsf(_b0);
	}

	if (limit > 0.0f) {
		_z3 = math::constrain(_z3, -limit, limit);
	}
}

float
Adrc::applyLimits(float u_new, float dt)
{
	float u = math::constrain(u_new, _u_min, _u_max);

	if (_u_rate_max > 0.0f) {
		u = math::constrain(u, _u_prev - _u_rate_max * dt, _u_prev + _u_rate_max * dt);
	}

	_u_prev = u;
	return u;
}

void
Adrc::updateNotch(float dt)
{
	const float fs = 1.0f / dt;

	/* recompute coefficients only when the sample rate changes by more than 1% */
	if (fabsf(fs - _notch_fs) > 0.01f * fs) {
		const float w0 = 2.0f * kPiF * _notch_hz;
		const float q = (_notch_hz / _notch_bw);
		const float alpha = sinf(w0 / fs) / (2.0f * q);
		const float cosw0 = cosf(w0 / fs);
		const float a0 = 1.0f + alpha;

		_notch_b0 = 1.0f / a0;
		_notch_b1 = -2.0f * cosw0 / a0;
		_notch_b2 = 1.0f / a0;
		_notch_a1 = -2.0f * cosw0 / a0;
		_notch_a2 = (1.0f - alpha) / a0;
		_notch_fs = fs;
	}
}

float
Adrc::applyNotch(float x)
{
	const float y = _notch_b0 * x + _notch_b1 * _notch_x1 + _notch_b2 * _notch_x2
			- _notch_a1 * _notch_y1 - _notch_a2 * _notch_y2;

	_notch_x2 = _notch_x1;
	_notch_x1 = x;
	_notch_y2 = _notch_y1;
	_notch_y1 = y;

	return y;
}

float
Adrc::updateNlsef()
{
	_e1 = _v1 - _z1;
	_e2 = _v2 - _z2;

	switch (_ctrl_law) {
	case 2:
		/* linear law: PD-like, easiest to tune first */
		return _nlsef_beta1 * _e1 + _nlsef_beta2 * _e2;

	default:
		/* nonlinear law (Han) */
		return _nlsef_beta1 * fal(_e1, _nlsef_alpha1, _delta)
		       + _nlsef_beta2 * fal(_e2, _nlsef_alpha2, _delta);
	}
}

float
Adrc::fhan(float x1, float x2, float r, float h)
{
	const float d = r * h * h;
	const float a0 = h * x2;
	const float y = x1 + a0;
	const float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
	const float a2 = a0 + signf(y) * (a1 - d) * 0.5f;
	const float a = (a0 + y) * fsg(y, d) + a2 * (1.0f - fsg(y, d));

	return -r * (a / d) * fsg(y, d) - r * signf(a) * (1.0f - fsg(a, d));
}

float
Adrc::fal(float e, float alpha, float delta)
{
	if (delta <= 0.0f) {
		return signf(e) * powf(fabsf(e), alpha);
	}

	return (fabsf(e) <= delta) ? e / powf(delta, alpha - 1.0f) : signf(e) * powf(fabsf(e), alpha);
}

float
Adrc::fsg(float x, float d)
{
	return (signf(x + d) - signf(x - d)) * 0.5f;
}

float
Adrc::signf(float x)
{
	return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}
