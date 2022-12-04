#pragma once

#include <SpeedValue.h>
#include <etsi_its_cam_msgs/SpeedValue.h>
#include <primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SpeedValue convert_SpeedValuetoRos(const SpeedValue_t& _SpeedValue_in)
	{
		etsi_its_cam_msgs::SpeedValue SpeedValue_out;
		convert_toRos(_SpeedValue_in, SpeedValue_out.value);
		return SpeedValue_out;
	}
}