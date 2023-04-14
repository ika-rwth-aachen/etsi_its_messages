#pragma once

#include <etsi_its_cam_coding/LongitudinalAccelerationValue.h>
#include <etsi_its_cam_msgs/LongitudinalAccelerationValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::LongitudinalAccelerationValue convert_LongitudinalAccelerationValuetoRos(const LongitudinalAccelerationValue_t& _LongitudinalAccelerationValue_in)
	{
		etsi_its_cam_msgs::LongitudinalAccelerationValue LongitudinalAccelerationValue_out;
		convert_toRos(_LongitudinalAccelerationValue_in, LongitudinalAccelerationValue_out.value);
		return LongitudinalAccelerationValue_out;
	}
	LongitudinalAccelerationValue_t convert_LongitudinalAccelerationValuetoC(const etsi_its_cam_msgs::LongitudinalAccelerationValue& _LongitudinalAccelerationValue_in)
	{
		LongitudinalAccelerationValue_t LongitudinalAccelerationValue_out;
		memset(&LongitudinalAccelerationValue_out, 0, sizeof(LongitudinalAccelerationValue_t));
		convert_toC(_LongitudinalAccelerationValue_in.value, LongitudinalAccelerationValue_out);
		return LongitudinalAccelerationValue_out;
	}
}