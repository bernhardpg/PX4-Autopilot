// TODO: Add header

#pragma once

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

enum signal_type
{
	TYPE_STEP_SIGNAL = 0,
	TYPE_2_1_1 = 1
};

class SystemIdController : public ModuleParams
{
public:
	SystemIdController();
	~SystemIdController() = default;

	bool is_active() {return _is_active;}
	float get_input(){return _input;}

	void update(float ref_value);
private:
	bool _should_run = true;
	bool _is_active = false; // will generate a step on roll
	hrt_abstime _init_time;
	hrt_abstime _sysid_start_time;

	hrt_abstime _sysid_duration;
	hrt_abstime _time_before_start = 2 * 1e6;
	hrt_abstime _time_after_end = 2 * 1e6;
	float _ref_value;

	hrt_abstime _delay_before_start = 45 * 1e6; // us
	hrt_abstime _step_length = 0.25 * 1e6; // us
	float _step_amplitude = 0.3;

	float _input; // Number between -1 and 1

	void sysid_activate(float ref_value);
	void sysid_deactivate();

	signal_type _signal_type;

	// Signal generators
	float generate_signal_step(float amplitude, float step_length);
	float generate_2_1_1(float ref_value, float amplitude, float step_length, bool inverted);

	//DEFINE_PARAMETERS( // TODO add parameters
	//	(ParamBool<px4::params::WV_EN>) _param_wv_en,
	//	(ParamFloat<px4::params::WV_ROLL_MIN>) _param_wv_roll_min,
	//	(ParamFloat<px4::params::WV_GAIN>) _param_wv_gain,
	//	(ParamFloat<px4::params::WV_YRATE_MAX>) _param_wv_yrate_max
	//)

};
