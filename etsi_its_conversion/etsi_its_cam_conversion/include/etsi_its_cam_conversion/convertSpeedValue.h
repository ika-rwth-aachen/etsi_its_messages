#pragma once

#include <etsi_its_cam_coding/SpeedValue.h>
#include <etsi_its_cam_msgs/SpeedValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SpeedValue convert_SpeedValuetoRos(const SpeedValue_t& _SpeedValue_in)
	{
		etsi_its_cam_msgs::SpeedValue SpeedValue_out;
		convert_toRos(_SpeedValue_in, SpeedValue_out.value);
		return SpeedValue_out;
	}
	SpeedValue_t convert_SpeedValuetoC(const etsi_its_cam_msgs::SpeedValue& _SpeedValue_in)
	{
		SpeedValue_t SpeedValue_out;
		memset(&SpeedValue_out, 0, sizeof(SpeedValue_t));
		convert_toC(_SpeedValue_in.value, SpeedValue_out);
		return SpeedValue_out;
	}
}