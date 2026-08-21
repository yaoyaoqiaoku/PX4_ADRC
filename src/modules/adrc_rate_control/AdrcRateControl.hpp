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

#pragma once

#include "Adrc.hpp"

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/adrc_status.h>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

/**
 * ADRC-based multicopter angular rate controller.
 *
 * Replacement for the stock PID rate controller (mc_rate_control), which is
 * disabled in the build. The module interface mirrors mc_rate_control so the
 * rest of the control pipeline is untouched.
 */
class AdrcRateControl : public ModuleBase<AdrcRateControl>, public ModuleParams, public px4::WorkItem
{
public:
	AdrcRateControl(bool vtol = false);
	~AdrcRateControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	void parameters_updated();
	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt);

	Adrc _adrc[3]; ///< roll / pitch / yaw ADRC controllers

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_controls_status_s> _actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::PublicationMulti<rate_ctrl_status_s> _controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub;
	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub;
	uORB::Publication<adrc_status_s> _adrc_status_pub{ORB_ID(adrc_status)};

	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_status_s _vehicle_status{};

	bool _landed{true};
	bool _armed_prev{false};

	/* runtime mode validation (keep-last-valid, mirrors mc_adrc_control):
	 * ADRC_ESO_MODE / ADRC_CTRL_LAW are validated against their legal sets on
	 * every parameter update; an invalid value keeps the last valid mode and
	 * warns once instead of silently switching branches mid-flight. */
	int  _last_eso_mode{1};		///< last valid ADRC_ESO_MODE (0=classic, 1=LADRC)
	int  _last_ctrl_law{2};		///< last valid ADRC_CTRL_LAW (1=nonlinear NLSEF, 2=linear)
	bool _invalid_eso_mode_warned{false};
	bool _invalid_ctrl_law_warned{false};
	bool _mode_changed{false};	///< ESO mode / control law changed via param update (reset states in Run)

	hrt_abstime _last_run{0};

	perf_counter_t _loop_perf; ///< loop duration performance counter

	matrix::Vector3f _acro_rate_max;   ///< max attitude rates in acro mode
	matrix::Vector3f _rates_setpoint{};
	matrix::Vector3f _thrust_setpoint{};

	float _battery_status_scale{0.0f};
	float _energy_integration_time{0.0f};
	float _control_energy[4] {};

	/* control-allocator saturation feedback (anti-windup, mirrors stock PID) */
	bool  _allocator_sat_pos[3] {};	///< torque saturated in positive direction per axis
	bool  _allocator_sat_neg[3] {};	///< torque saturated in negative direction per axis
	float _unallocated_torque[3] {};	///< torque the allocator could not achieve per axis
	float _torque_cmd_prev[3] {};		///< torque actually sent to the allocator last cycle

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ADRC_CTRL_LAW>) _param_adrc_ctrl_law,
		(ParamInt<px4::params::ADRC_ESO_MODE>) _param_adrc_eso_mode,

		(ParamFloat<px4::params::ADRC_ROLL_R>) _param_adrc_roll_r,
		(ParamFloat<px4::params::ADRC_ROLL_B0>) _param_adrc_roll_b0,
		(ParamFloat<px4::params::ADRC_ROLL_DELTA>) _param_adrc_roll_delta,
		(ParamFloat<px4::params::ADRC_ROLL_B01>) _param_adrc_roll_b01,
		(ParamFloat<px4::params::ADRC_ROLL_B02>) _param_adrc_roll_b02,
		(ParamFloat<px4::params::ADRC_ROLL_B03>) _param_adrc_roll_b03,
		(ParamFloat<px4::params::ADRC_ROLL_NB1>) _param_adrc_roll_nb1,
		(ParamFloat<px4::params::ADRC_ROLL_NB2>) _param_adrc_roll_nb2,
		(ParamFloat<px4::params::ADRC_ROLL_A1>) _param_adrc_roll_a1,
		(ParamFloat<px4::params::ADRC_ROLL_A2>) _param_adrc_roll_a2,
		(ParamFloat<px4::params::ADRC_ROLL_UMX>) _param_adrc_roll_umx,
		(ParamFloat<px4::params::ADRC_ROLL_UMN>) _param_adrc_roll_umn,
		(ParamFloat<px4::params::ADRC_ROLL_ESO_W>) _param_adrc_roll_eso_w,
		(ParamFloat<px4::params::ADRC_ROLL_CW>) _param_adrc_roll_cw,
		(ParamFloat<px4::params::ADRC_ROLL_FLT>) _param_adrc_roll_flt,
		(ParamFloat<px4::params::ADRC_ROLL_NF>) _param_adrc_roll_nf,
		(ParamFloat<px4::params::ADRC_ROLL_NBW>) _param_adrc_roll_nbw,
		(ParamFloat<px4::params::ADRC_ROLL_TAU>) _param_adrc_roll_tau,
		(ParamFloat<px4::params::ADRC_ROLL_Z3MAX>) _param_adrc_roll_z3max,
		(ParamFloat<px4::params::ADRC_ROLL_FF>) _param_adrc_roll_ff,
		(ParamFloat<px4::params::ADRC_ROLL_KI>) _param_adrc_roll_ki,
		(ParamFloat<px4::params::ADRC_ROLL_RAMP>) _param_adrc_roll_ramp,
		(ParamFloat<px4::params::ADRC_ROLL_GAMMA>) _param_adrc_roll_gamma,
		(ParamFloat<px4::params::ADRC_ROLL_SPS>) _param_adrc_roll_sps,
		(ParamFloat<px4::params::ADRC_ROLL_LZ3>) _param_adrc_roll_lz3,
		(ParamFloat<px4::params::ADRC_ROLL_AF>) _param_adrc_roll_af,
		(ParamFloat<px4::params::ADRC_ROLL_ILIM>) _param_adrc_roll_ilim,

		(ParamFloat<px4::params::ADRC_PITCH_R>) _param_adrc_pitch_r,
		(ParamFloat<px4::params::ADRC_PITCH_B0>) _param_adrc_pitch_b0,
		(ParamFloat<px4::params::ADRC_PITCH_DELTA>) _param_adrc_pitch_delta,
		(ParamFloat<px4::params::ADRC_PITCH_B01>) _param_adrc_pitch_b01,
		(ParamFloat<px4::params::ADRC_PITCH_B02>) _param_adrc_pitch_b02,
		(ParamFloat<px4::params::ADRC_PITCH_B03>) _param_adrc_pitch_b03,
		(ParamFloat<px4::params::ADRC_PITCH_NB1>) _param_adrc_pitch_nb1,
		(ParamFloat<px4::params::ADRC_PITCH_NB2>) _param_adrc_pitch_nb2,
		(ParamFloat<px4::params::ADRC_PITCH_A1>) _param_adrc_pitch_a1,
		(ParamFloat<px4::params::ADRC_PITCH_A2>) _param_adrc_pitch_a2,
		(ParamFloat<px4::params::ADRC_PITCH_UMX>) _param_adrc_pitch_umx,
		(ParamFloat<px4::params::ADRC_PITCH_UMN>) _param_adrc_pitch_umn,
		(ParamFloat<px4::params::ADRC_PITCH_ESO_W>) _param_adrc_pitch_eso_w,
		(ParamFloat<px4::params::ADRC_PITCH_CW>) _param_adrc_pitch_cw,
		(ParamFloat<px4::params::ADRC_PITCH_FLT>) _param_adrc_pitch_flt,
		(ParamFloat<px4::params::ADRC_PITCH_NF>) _param_adrc_pitch_nf,
		(ParamFloat<px4::params::ADRC_PITCH_NBW>) _param_adrc_pitch_nbw,
		(ParamFloat<px4::params::ADRC_PITCH_TAU>) _param_adrc_pitch_tau,
		(ParamFloat<px4::params::ADRC_PITCH_Z3MAX>) _param_adrc_pitch_z3max,
		(ParamFloat<px4::params::ADRC_PITCH_FF>) _param_adrc_pitch_ff,
		(ParamFloat<px4::params::ADRC_PITCH_KI>) _param_adrc_pitch_ki,
		(ParamFloat<px4::params::ADRC_PITCH_RAMP>) _param_adrc_pitch_ramp,
		(ParamFloat<px4::params::ADRC_PITCH_GAMMA>) _param_adrc_pitch_gamma,
		(ParamFloat<px4::params::ADRC_PITCH_SPS>) _param_adrc_pitch_sps,
		(ParamFloat<px4::params::ADRC_PITCH_LZ3>) _param_adrc_pitch_lz3,
		(ParamFloat<px4::params::ADRC_PITCH_AF>) _param_adrc_pitch_af,
		(ParamFloat<px4::params::ADRC_PITCH_ILIM>) _param_adrc_pitch_ilim,

		(ParamFloat<px4::params::ADRC_YAW_R>) _param_adrc_yaw_r,
		(ParamFloat<px4::params::ADRC_YAW_B0>) _param_adrc_yaw_b0,
		(ParamFloat<px4::params::ADRC_YAW_DELTA>) _param_adrc_yaw_delta,
		(ParamFloat<px4::params::ADRC_YAW_B01>) _param_adrc_yaw_b01,
		(ParamFloat<px4::params::ADRC_YAW_B02>) _param_adrc_yaw_b02,
		(ParamFloat<px4::params::ADRC_YAW_B03>) _param_adrc_yaw_b03,
		(ParamFloat<px4::params::ADRC_YAW_NB1>) _param_adrc_yaw_nb1,
		(ParamFloat<px4::params::ADRC_YAW_NB2>) _param_adrc_yaw_nb2,
		(ParamFloat<px4::params::ADRC_YAW_A1>) _param_adrc_yaw_a1,
		(ParamFloat<px4::params::ADRC_YAW_A2>) _param_adrc_yaw_a2,
		(ParamFloat<px4::params::ADRC_YAW_UMX>) _param_adrc_yaw_umx,
		(ParamFloat<px4::params::ADRC_YAW_UMN>) _param_adrc_yaw_umn,
		(ParamFloat<px4::params::ADRC_YAW_ESO_W>) _param_adrc_yaw_eso_w,
		(ParamFloat<px4::params::ADRC_YAW_CW>) _param_adrc_yaw_cw,
		(ParamFloat<px4::params::ADRC_YAW_FLT>) _param_adrc_yaw_flt,
		(ParamFloat<px4::params::ADRC_YAW_NF>) _param_adrc_yaw_nf,
		(ParamFloat<px4::params::ADRC_YAW_NBW>) _param_adrc_yaw_nbw,
		(ParamFloat<px4::params::ADRC_YAW_TAU>) _param_adrc_yaw_tau,
		(ParamFloat<px4::params::ADRC_YAW_Z3MAX>) _param_adrc_yaw_z3max,
		(ParamFloat<px4::params::ADRC_YAW_FF>) _param_adrc_yaw_ff,
		(ParamFloat<px4::params::ADRC_YAW_KI>) _param_adrc_yaw_ki,
		(ParamFloat<px4::params::ADRC_YAW_RAMP>) _param_adrc_yaw_ramp,
		(ParamFloat<px4::params::ADRC_YAW_GAMMA>) _param_adrc_yaw_gamma,
		(ParamFloat<px4::params::ADRC_YAW_SPS>) _param_adrc_yaw_sps,
		(ParamFloat<px4::params::ADRC_YAW_LZ3>) _param_adrc_yaw_lz3,
		(ParamFloat<px4::params::ADRC_YAW_AF>) _param_adrc_yaw_af,
		(ParamFloat<px4::params::ADRC_YAW_ILIM>) _param_adrc_yaw_ilim,

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en
	)
};
