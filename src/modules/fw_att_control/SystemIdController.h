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

	void activate() {_is_active = true;}
	void deactivate() {_is_active = false;}
	bool is_active() {return _is_active;}

	void update();

private:

	bool _is_active = false;
	uint64_t _start_time;
	uint64_t _delay_before_start = 1000000; // TODO change

	//DEFINE_PARAMETERS( // TODO add parameters
	//	(ParamBool<px4::params::WV_EN>) _param_wv_en,
	//	(ParamFloat<px4::params::WV_ROLL_MIN>) _param_wv_roll_min,
	//	(ParamFloat<px4::params::WV_GAIN>) _param_wv_gain,
	//	(ParamFloat<px4::params::WV_YRATE_MAX>) _param_wv_yrate_max
	//)

};
