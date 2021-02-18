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
	int _init_time;
	int _sysid_start_time;

	int _delay_before_start = 50 * 1000000; // TODO change
	float _input_low = 0.2;
	float _input_high = 0.4;
	int _step_length = 1* 1000000; // us
	int _sysid_duration = _step_length * 2;

	float _input; // Number between -1 and 1

	void sysid_activate();
	void sysid_deactivate();

	int time_since_init(){return hrt_absolute_time() - _init_time;}
	int time_elapsed(){return hrt_absolute_time() - _sysid_start_time;}
	void set_sysid_start_time(){_sysid_start_time = hrt_absolute_time();}
	float generate_signal_step();

	//DEFINE_PARAMETERS( // TODO add parameters
	//	(ParamBool<px4::params::WV_EN>) _param_wv_en,
	//	(ParamFloat<px4::params::WV_ROLL_MIN>) _param_wv_roll_min,
	//	(ParamFloat<px4::params::WV_GAIN>) _param_wv_gain,
	//	(ParamFloat<px4::params::WV_YRATE_MAX>) _param_wv_yrate_max
	//)

};
