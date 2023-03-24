#pragma once

#include <etsi_its_cam_coding/VerticalAccelerationValue.h>
#include <etsi_its_cam_msgs/VerticalAccelerationValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::VerticalAccelerationValue convert_VerticalAccelerationValuetoRos(const VerticalAccelerationValue_t& _VerticalAccelerationValue_in)
	{
		etsi_its_cam_msgs::VerticalAccelerationValue VerticalAccelerationValue_out;
		convert_toRos(_VerticalAccelerationValue_in, VerticalAccelerationValue_out.value);
		return VerticalAccelerationValue_out;
	}
}