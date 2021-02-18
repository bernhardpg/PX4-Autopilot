#include "SystemIdController.h"

SystemIdController::SystemIdController() :
	ModuleParams(nullptr)
{
	_init_time = hrt_absolute_time();
	PX4_INFO("%d Sysid initialized", (int)_init_time);
}

void SystemIdController::sysid_activate()
{
	_is_active = true;
	set_sysid_start_time();
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
	if (!_is_active && _should_run && time_since_init() > _delay_before_start)
	{
		sysid_activate();
	}

	if (_is_active)
	{
		_input = generate_signal_step();
		//PX4_INFO("Input: %f", (double)_input);

		if (time_elapsed() > _sysid_duration)
		{
			sysid_deactivate();
		}
	}

}

float SystemIdController::generate_signal_step()
{
	if (time_elapsed() < _step_length)
	{
		return 0;
	}
	else if (time_elapsed() < _step_length * 2)
	{
		return _input_high;
	}
	return 0;
}
