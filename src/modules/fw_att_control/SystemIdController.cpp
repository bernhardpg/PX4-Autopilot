#include "SystemIdController.h"

SystemIdController::SystemIdController() :
	ModuleParams(nullptr)
{
	_init_time = hrt_absolute_time();
	_sysid_duration = _step_length;
	PX4_INFO("%d Sysid initialized", (int)_init_time);
}

void SystemIdController::sysid_activate()
{
	_is_active = true;
	_sysid_start_time = hrt_absolute_time();
	PX4_INFO("%d Sysid maneuver starting", (int)_sysid_start_time);
}

void SystemIdController::sysid_deactivate()
{
	_is_active = false;
	_should_run = false;
	PX4_INFO("Sysid maneuver finished");
}

void SystemIdController::update()
{
	// Activate after a certain time
	if (!_is_active && _should_run && hrt_elapsed_time(&_init_time) > _delay_before_start)
	{
		sysid_activate();
	}

	if (_is_active)
	{
		_input = generate_signal_step(_step_amplitude, _step_length);
		//PX4_INFO("Input: %f", (double)_input);

		if (hrt_elapsed_time(&_sysid_start_time) > _sysid_duration)
			sysid_deactivate();
	}

}

float SystemIdController::generate_signal_step(float amplitude, float step_length)
{
	if (hrt_elapsed_time(&_sysid_start_time) < step_length)
	{
		return amplitude;
	}
	return 0;
}


