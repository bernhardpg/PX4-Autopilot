// TODO: Add header

#pragma once

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

enum signal_type_t
{
	TYPE_2_1_1 = 0,
	TYPE_SWEEP = 1,
	TYPE_STEP_SIGNAL = 2,
};

enum actuator_index_t
{
	SYSID_AXIS_ROLL = 0,
	SYSID_AXIS_PITCH = 1
};

class SystemIdController : public ModuleParams // TODO change name
{
public:
	SystemIdController();
	~SystemIdController() = default;

	void sysid_activate(float ref_value);
	void sysid_reset(){_should_run = true;}
	void sysid_deactivate(); // TODO change name

	bool is_activated() {return _param_sysid_enable.get();}
	bool maneuver_is_active() {return _is_active;}
	actuator_index_t get_active_axis(){return _active_axis;}
	float get_output(){return _output;}

	void update();
	void parameters_update();
private:
	bool _should_run = true;
	bool _is_active = false; // will generate a step on roll
	hrt_abstime _init_time;
	hrt_abstime _sysid_start_time;

	actuator_index_t _active_axis;
	hrt_abstime _sysid_duration{};
	hrt_abstime _idle_time_before;
	hrt_abstime _idle_time_after;
	hrt_abstime _step_length;
	float _step_amplitude;
	bool _invert_signal = false;
	bool _start_up = true;

	float _ref_value; // The signals will be generated around this value

	float _output; // Number between -1 and 1

	signal_type_t _signal_type; // Defaults to 2-1-1
	bool _sweep_do_both_sides;

	// Signal generators
	float generate_signal_step(float amplitude, float step_length);
	float generate_2_1_1(
		float ref_value, float amplitude, float step_length, bool inverted, bool start_up
		);
	float generate_sweep(
		float ref_value, float amplitude, float step_length, bool inverted, bool start_up, bool do_both_sides
		);
	void update_sysid_duration();

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< notification of parameter updates */

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SYSID_ENABLE>) _param_sysid_enable,
		(ParamFloat<px4::params::SYSID_IDLE_T_B>) _param_sysid_idle_time_before,
		(ParamFloat<px4::params::SYSID_IDLE_T_A>) _param_sysid_idle_time_after,
		(ParamInt<px4::params::SYSID_ACT_AX>) _param_sysid_active_axis,
		(ParamFloat<px4::params::SYSID_STEP_AMPL>) _param_sysid_step_amplitude,
		(ParamFloat<px4::params::SYSID_STEP_LNGTH>) _param_sysid_step_length,
		(ParamBool<px4::params::SYSID_AUTO_INV>) _param_sysid_auto_invert,
		(ParamBool<px4::params::SYSID_FSTART_UP>) _param_sysid_force_start_up,
		(ParamInt<px4::params::SYSID_SIG_TYPE>) _param_sysid_signal_type,
		(ParamBool<px4::params::SYSID_SWP_BOTH>) _param_sysid_sweep_do_both_sides
	)
};
