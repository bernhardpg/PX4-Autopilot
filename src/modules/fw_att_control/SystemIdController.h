// TODO: Add header

#pragma once

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

class SystemIdController : public ModuleParams
{
public:
	SystemIdController();
	~SystemIdController() = default;

	bool is_active() {return _is_active;}
	float get_input(){return _input;}

	void update();
private:
	bool _should_run = true;
	bool _is_active = false; // will generate a step on roll
	hrt_abstime _init_time;
	hrt_abstime _sysid_start_time;

	hrt_abstime _sysid_duration;
	hrt_abstime _delay_before_start = 50 * 1e6;
	hrt_abstime _step_length = 1 * 1000000; // us
	int _step_amplitude = 0.3;

	float _input; // Number between -1 and 1

	void sysid_activate();
	void sysid_deactivate();

	float generate_signal_step(float amplitude, float step_length);

	//DEFINE_PARAMETERS( // TODO add parameters
	//	(ParamBool<px4::params::WV_EN>) _param_wv_en,
	//	(ParamFloat<px4::params::WV_ROLL_MIN>) _param_wv_roll_min,
	//	(ParamFloat<px4::params::WV_GAIN>) _param_wv_gain,
	//	(ParamFloat<px4::params::WV_YRATE_MAX>) _param_wv_yrate_max
	//)

};
