#include "SystemIdController.h"

SystemIdController::SystemIdController() :
	ModuleParams(nullptr)
{

	_start_time = hrt_absolute_time();
	PX4_INFO("SysId start time: %d\n", (int)_start_time);
}
