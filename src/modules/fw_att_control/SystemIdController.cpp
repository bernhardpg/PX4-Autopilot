#include "SystemIdController.h"

SystemIdController::SystemIdController() :
	ModuleParams(nullptr)
{
	_init_time = hrt_absolute_time();
	_signal_type = TYPE_2_1_1; // NOTE: Can be expanded to parameters if needed
	_active_axis = ROLL;

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
	_sysid_duration += _idle_time_before + _idle_time_after;

	parameters_update();

	PX4_INFO("%d Sysid initialized", (int)_init_time);
}

void SystemIdController::sysid_activate(float ref_value)
{
	// Only activate if the sysid maneuver has not yet been run
	if (_should_run)
	{
		_is_active = true;
		_sysid_start_time = hrt_absolute_time();
		_ref_value = ref_value;
		PX4_INFO("%d Sysid maneuver starting", (int)_sysid_start_time);
	}
}

void SystemIdController::parameters_update()
{
	// only update parameters if they changed
	bool params_updated = _parameter_update_sub.updated();

	// check for parameter updates
	if (params_updated) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

	}

	/* Sysid maneuver parameters */
	_idle_time_before = _param_sysid_idle_time_before.get();
	_idle_time_after = _param_sysid_idle_time_after.get();
	_step_length = _param_sysid_step_length.get();
	_step_amplitude = _param_sysid_step_amplitude.get();
	_active_axis = (actuator_index)_param_sysid_active_axis.get();
}

void SystemIdController::sysid_deactivate()
{
	_is_active = false;
	_should_run = false;
	PX4_INFO("Sysid maneuver finished");
}

void SystemIdController::update(float ref_value)
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

	// Deactivate the sysid maneuver to prevent it from running again
	if (hrt_elapsed_time(&_sysid_start_time) > _sysid_duration)
	{
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
	if (hrt_elapsed_time(&_sysid_start_time) < _idle_time_before)
	{
		return ref_value;
	}
	else if (hrt_elapsed_time(&_sysid_start_time) < _idle_time_before + step_length * 2)
	{
		return ref_value + sign * amplitude;
	}
	else if (hrt_elapsed_time(&_sysid_start_time) < _idle_time_before + step_length * 3)
	{
		return ref_value + sign * amplitude * (-1);
	}
	else if (hrt_elapsed_time(&_sysid_start_time) < _idle_time_before + step_length * 4)
	{
		return ref_value + sign * amplitude;
	}
	return ref_value;
}


