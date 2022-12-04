#pragma once

#include <LongitudinalAccelerationValue.h>
#include <etsi_its_cam_msgs/LongitudinalAccelerationValue.h>
#include <primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::LongitudinalAccelerationValue convert_LongitudinalAccelerationValuetoRos(const LongitudinalAccelerationValue_t& _LongitudinalAccelerationValue_in)
	{
		etsi_its_cam_msgs::LongitudinalAccelerationValue LongitudinalAccelerationValue_out;
		convert_toRos(_LongitudinalAccelerationValue_in, LongitudinalAccelerationValue_out.value);
		return LongitudinalAccelerationValue_out;
	}
}