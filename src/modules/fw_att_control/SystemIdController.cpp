#include "SystemIdController.h"

SystemIdController::SystemIdController() :
	ModuleParams(nullptr)
{
	_init_time = hrt_absolute_time();
	_signal_type = TYPE_2_1_1;
	_active_actuator_index = ROLL;

	switch (_signal_type)
	{
	case TYPE_STEP_SIGNAL:
		_sysid_duration = _step_length;
		break;

	case TYPE_2_1_1:
	default:
		_sysid_duration = _step_length * 4;
		break;
	}
	_sysid_duration += _time_before_start + _time_after_end;

	PX4_INFO("%d Sysid initialized", (int)_init_time);
	PX4_INFO("Sysid duration %d", (int)_sysid_duration);
}

void SystemIdController::sysid_activate(float ref_value)
{
	_is_active = true;
	_sysid_start_time = hrt_absolute_time();
	_ref_value = ref_value;
	PX4_INFO("%d Sysid maneuver starting", (int)_sysid_start_time);
}

void SystemIdController::sysid_deactivate()
{
	_is_active = false;
	_should_run = false;
	PX4_INFO("Sysid maneuver finished");
}

void SystemIdController::update(float ref_value)
{
	// Activate after a certain time
	if (!_is_active && _should_run && hrt_elapsed_time(&_init_time) > _delay_before_start)
	{
		sysid_activate(ref_value);
	}

	if (_is_active)
	{
		switch (_signal_type)
		{
		case TYPE_2_1_1:
			_output = generate_2_1_1(_ref_value, _step_amplitude, _step_length, false);
			break;
		case TYPE_STEP_SIGNAL:
		default:
			_output = generate_signal_step(_step_amplitude, _step_length);
			break;
		}
		//PX4_INFO("output: %f", (double)_output);

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

float SystemIdController::generate_2_1_1(float ref_value, float amplitude, float step_length, bool inverted)
{
	float sign = inverted ? -1 : 1;
	if (hrt_elapsed_time(&_sysid_start_time) < _time_before_start)
	{
		return ref_value;
	}
	else if (hrt_elapsed_time(&_sysid_start_time) < _time_before_start + step_length * 2)
	{
		return ref_value + sign * amplitude;
	}
	else if (hrt_elapsed_time(&_sysid_start_time) < _time_before_start + step_length * 3)
	{
		return ref_value + sign * amplitude * (-1);
	}
	else if (hrt_elapsed_time(&_sysid_start_time) < _time_before_start + step_length * 4)
	{
		return ref_value + sign * amplitude;
	}
	return ref_value;
}


