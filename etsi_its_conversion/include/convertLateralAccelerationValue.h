#pragma once

#include <LateralAccelerationValue.h>
#include <etsi_its_cam_msgs/LateralAccelerationValue.h>
#include <convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::LateralAccelerationValue convert_LateralAccelerationValuetoRos(const LateralAccelerationValue_t& _LateralAccelerationValue_in)
	{
		etsi_its_cam_msgs::LateralAccelerationValue LateralAccelerationValue_out;
		convert_toRos(_LateralAccelerationValue_in, LateralAccelerationValue_out.value);
		return LateralAccelerationValue_out;
	}
}