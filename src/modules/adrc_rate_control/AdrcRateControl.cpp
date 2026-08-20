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
 * @file AdrcRateControl.cpp
 *
 * ADRC-based multicopter angular rate controller.
 */

#include "AdrcRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>

using namespace matrix;
using namespace time_literals;
using math::radians;

AdrcRateControl::AdrcRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

AdrcRateControl::~AdrcRateControl()
{
	perf_free(_loop_perf);
}

bool
AdrcRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
AdrcRateControl::parameters_updated()
{
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	auto configure = [](Adrc &adrc, int law, int eso_mode, float eso_w, float ctrl_w,
	const float p[11]) {
		adrc.setControlLaw(law);
		adrc.setEsoMode(eso_mode);
		adrc.setObserverBandwidth(eso_w);
		adrc.setControllerBandwidth(ctrl_w);
		adrc.setFeedbackFilter(p[0]);
		adrc.setNotch(p[1], p[2]);
		adrc.setActuatorTau(p[3]);
		adrc.setZ3Max(p[4]);
		adrc.setFeedforward(p[5]);
		adrc.setIntegralGain(p[6]);
		adrc.setOutputRateLimit(p[7]);
		adrc.setGamma(p[8]);
		adrc.setSetpointFilter(p[9]);
		adrc.setIntegratorLimit(p[10]);
	};

	const float roll_hardening[11] = {
		_param_adrc_roll_flt.get(), _param_adrc_roll_nf.get(), _param_adrc_roll_nbw.get(),
		_param_adrc_roll_tau.get(), _param_adrc_roll_z3max.get(), _param_adrc_roll_ff.get(),
		_param_adrc_roll_ki.get(), _param_adrc_roll_ramp.get(),
		_param_adrc_roll_gamma.get(), _param_adrc_roll_sps.get(),
		_param_adrc_roll_ilim.get()
	};
	const float pitch_hardening[11] = {
		_param_adrc_pitch_flt.get(), _param_adrc_pitch_nf.get(), _param_adrc_pitch_nbw.get(),
		_param_adrc_pitch_tau.get(), _param_adrc_pitch_z3max.get(), _param_adrc_pitch_ff.get(),
		_param_adrc_pitch_ki.get(), _param_adrc_pitch_ramp.get(),
		_param_adrc_pitch_gamma.get(), _param_adrc_pitch_sps.get(),
		_param_adrc_pitch_ilim.get()
	};
	const float yaw_hardening[11] = {
		_param_adrc_yaw_flt.get(), _param_adrc_yaw_nf.get(), _param_adrc_yaw_nbw.get(),
		_param_adrc_yaw_tau.get(), _param_adrc_yaw_z3max.get(), _param_adrc_yaw_ff.get(),
		_param_adrc_yaw_ki.get(), _param_adrc_yaw_ramp.get(),
		_param_adrc_yaw_gamma.get(), _param_adrc_yaw_sps.get(),
		_param_adrc_yaw_ilim.get()
	};

	configure(_adrc[0], _param_adrc_ctrl_law.get(), _param_adrc_eso_mode.get(),
		  _param_adrc_roll_eso_w.get(), _param_adrc_roll_cw.get(), roll_hardening);
	configure(_adrc[1], _param_adrc_ctrl_law.get(), _param_adrc_eso_mode.get(),
		  _param_adrc_pitch_eso_w.get(), _param_adrc_pitch_cw.get(), pitch_hardening);
	configure(_adrc[2], _param_adrc_ctrl_law.get(), _param_adrc_eso_mode.get(),
		  _param_adrc_yaw_eso_w.get(), _param_adrc_yaw_cw.get(), yaw_hardening);

	_adrc[0].setParameters(
		_param_adrc_roll_r.get(), _param_adrc_roll_b0.get(), _param_adrc_roll_delta.get(),
		_param_adrc_roll_b01.get(), _param_adrc_roll_b02.get(), _param_adrc_roll_b03.get(),
		_param_adrc_roll_nb1.get(), _param_adrc_roll_nb2.get(),
		_param_adrc_roll_a1.get(), _param_adrc_roll_a2.get(),
		_param_adrc_roll_umn.get(), _param_adrc_roll_umx.get());

	_adrc[1].setParameters(
		_param_adrc_pitch_r.get(), _param_adrc_pitch_b0.get(), _param_adrc_pitch_delta.get(),
		_param_adrc_pitch_b01.get(), _param_adrc_pitch_b02.get(), _param_adrc_pitch_b03.get(),
		_param_adrc_pitch_nb1.get(), _param_adrc_pitch_nb2.get(),
		_param_adrc_pitch_a1.get(), _param_adrc_pitch_a2.get(),
		_param_adrc_pitch_umn.get(), _param_adrc_pitch_umx.get());

	_adrc[2].setParameters(
		_param_adrc_yaw_r.get(), _param_adrc_yaw_b0.get(), _param_adrc_yaw_delta.get(),
		_param_adrc_yaw_b01.get(), _param_adrc_yaw_b02.get(), _param_adrc_yaw_b03.get(),
		_param_adrc_yaw_nb1.get(), _param_adrc_yaw_nb2.get(),
		_param_adrc_yaw_a1.get(), _param_adrc_yaw_a2.get(),
		_param_adrc_yaw_umn.get(), _param_adrc_yaw_umx.get());
}

void
AdrcRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check if parameters have changed */
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		/* guard against too small (< 0.125ms) and too large (> 20ms) dt's */
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		/* control-allocator saturation feedback for anti-windup (stock PID
		 * equivalent): when the allocator cannot achieve the last torque
		 * setpoint, freeze the ADRC integral in the saturated direction and
		 * feed the achieved torque into the ESO instead of the command */
		control_allocator_status_s control_allocator_status;

		if (_control_allocator_status_sub.update(&control_allocator_status)) {
			if (!control_allocator_status.torque_setpoint_achieved) {
				for (int i = 0; i < 3; i++) {
					_allocator_sat_pos[i] = control_allocator_status.unallocated_torque[i] > 1e-6f;
					_allocator_sat_neg[i] = control_allocator_status.unallocated_torque[i] < -1e-6f;
					_unallocated_torque[i] = control_allocator_status.unallocated_torque[i];
				}

			} else {
				for (int i = 0; i < 3; i++) {
					_allocator_sat_pos[i] = false;
					_allocator_sat_neg[i] = false;
					_unallocated_torque[i] = 0.0f;
				}
			}
		}

		/* use rates setpoint topic */
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			/* generate the rate setpoint from sticks (ACRO mode) */
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				/* publish rate setpoint */
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
			}
		}

		/* run the ADRC rate controller */
		const bool armed = _vehicle_control_mode.flag_armed
				   && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

		/* full reset on the disarmed -> armed edge (seed ESO with current rates) */
		if (armed && !_armed_prev) {
			for (int i = 0; i < 3; i++) {
				_adrc[i].reset(rates(i));
			}
		}

		_armed_prev = armed;

		if (_vehicle_control_mode.flag_control_rates_enabled) {

			Vector3f att_control;

			for (int i = 0; i < 3; i++) {
				if (!armed) {
					/* keep states reset while disarmed */
					_adrc[i].reset(rates(i));

				} else if (_landed || _maybe_landed) {
					/* on the ground with motors running: only zero the disturbance
					 * estimate (mirrors the stock integrator reset), keep TD and
					 * velocity states so the controller can act during takeoff */
					_adrc[i].resetIntegral();
				}

				/* anti-windup: pass control-allocator saturation and the achieved
				 * torque into the controller before this cycle's update */
				_adrc[i].setSaturationStatus(_allocator_sat_pos[i], _allocator_sat_neg[i]);

				if (_allocator_sat_pos[i] || _allocator_sat_neg[i]) {
					const float scale = (_battery_status_scale > 0.f) ? _battery_status_scale : 1.0f;
					_adrc[i].setAppliedTorque(
						math::constrain(_torque_cmd_prev[i] - _unallocated_torque[i] / scale, -1.0f, 1.0f));
				}

				att_control(i) = _adrc[i].update(_rates_setpoint(i), rates(i), dt);
			}

			/* remember the torque command (controller units, pre battery-scale)
			 * for the control-allocator anti-windup on the next cycle */
			_torque_cmd_prev[0] = att_control(0);
			_torque_cmd_prev[1] = att_control(1);
			_torque_cmd_prev[2] = att_control(2);

			/* publish rate controller status (ADRC: total disturbance estimate) */
			rate_ctrl_status_s rate_ctrl_status{};
			rate_ctrl_status.rollspeed_integ = _adrc[0].getZ3();
			rate_ctrl_status.pitchspeed_integ = _adrc[1].getZ3();
			rate_ctrl_status.yawspeed_integ = _adrc[2].getZ3();
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			/* publish thrust and torque setpoints */
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.f;

			/* scale setpoints by battery status if enabled */
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

		} else {
			/* no control output this cycle: reset the allocator anti-windup
			 * state (the allocator sees no new torque setpoint) */
			_torque_cmd_prev[0] = _torque_cmd_prev[1] = _torque_cmd_prev[2] = 0.0f;
			_allocator_sat_pos[0] = _allocator_sat_pos[1] = _allocator_sat_pos[2] = false;
			_allocator_sat_neg[0] = _allocator_sat_neg[1] = _allocator_sat_neg[2] = false;
			_unallocated_torque[0] = _unallocated_torque[1] = _unallocated_torque[2] = 0.0f;
		}

		/* publish ADRC debug status unconditionally (also while disarmed) so the
		 * topic is always alive when the module is running */
		adrc_status_s adrc_status{};

		for (int i = 0; i < 3; i++) {
			adrc_status.z1[i] = _adrc[i].getZ1();
			adrc_status.z2[i] = _adrc[i].getZ2();
			adrc_status.z3[i] = _adrc[i].getZ3();
			adrc_status.v1[i] = _adrc[i].getV1();
			adrc_status.v2[i] = _adrc[i].getV2();
			adrc_status.u[i] = _adrc[i].getU();
			adrc_status.e1[i] = _adrc[i].getE1();
			adrc_status.e2[i] = _adrc[i].getE2();
			adrc_status.y[i] = _adrc[i].getY();
			adrc_status.ueso[i] = _adrc[i].getUEso();
			adrc_status.integ[i] = _adrc[i].getIntegral();
			adrc_status.z3raw[i] = _adrc[i].getZ3Raw();
			adrc_status.sp[i] = _adrc[i].getSp();
		}

		adrc_status.timestamp = hrt_absolute_time();
		_adrc_status_pub.publish(adrc_status);
	}

	perf_end(_loop_perf);
}

void
AdrcRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int
AdrcRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	AdrcRateControl *instance = new AdrcRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int
AdrcRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
AdrcRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter angular rate controller using ADRC (Active Disturbance Rejection Control).
It is a drop-in replacement for mc_rate_control: it takes rate setpoints (in acro mode via
`manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller is the sole angular rate controller (the stock PID mc_rate_control module
is disabled). Two ESO structures are available: classic ADRC (TD + nonlinear fal ESO,
ADRC_ESO_MODE=0) and LADRC (linear ESO + bandwidth parameterization, ADRC_ESO_MODE=1,
the default). Tuning parameters: ADRC_* per axis.

Real-aircraft hardening options (per axis): ADRC_*_FLT feedback low-pass,
	ADRC_*_NF/ADRC_*_NBW feedback notch, ADRC_*_TAU actuator-lag compensation in the
	ESO (when the observer pole-placement gains are all positive, i.e.
	1/3 < ESO_W*TAU < 1, this engages a 3rd-order augmented ESO that estimates
	the actuator state so the lag is no longer misread as disturbance; the
	actuator is modeled as a saturated first-order element with conditional
	integration at the torque rail [-1,1]), ADRC_*_Z3MAX disturbance-estimate
saturation (0 = auto 2*|b0|), ADRC_*_FF
rate-setpoint feed-forward, ADRC_*_KI anti-windup integral and ADRC_*_RAMP output
slew-rate limit. ADRC_*_GAMMA is the disturbance compensation ratio (FMT /
Jiachi Zou thesis; 1 = full compensation, lower = more robust on real
airframes) and ADRC_*_SPS smooths the rate setpoint ("poor man's TD") to avoid
exciting flexible modes. New options default to off/1.0 so the behavior is
unchanged unless enabled.

Control-allocator saturation feedback (stock PID equivalent) is always
enabled: when the allocator cannot achieve the torque setpoint, the ADRC
integral is frozen in the saturated direction and the ESO is fed the achieved
torque instead of the command, so saturation is never interpreted as
disturbance (anti-windup).

### Usage
adrc_rate_control <command> [arguments...]
 Commands:
   start [vtol]   Start the controller (vtol version if vtol argument provided)
   stop
   status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("adrc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int adrc_rate_control_main(int argc, char *argv[])
{
	return AdrcRateControl::main(argc, argv);
}
