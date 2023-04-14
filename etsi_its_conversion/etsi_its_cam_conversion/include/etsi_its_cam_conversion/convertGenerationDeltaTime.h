#pragma once

#include <etsi_its_cam_coding/GenerationDeltaTime.h>
#include <etsi_its_cam_msgs/GenerationDeltaTime.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_GenerationDeltaTimetoRos(const GenerationDeltaTime_t& _GenerationDeltaTime_in, etsi_its_cam_msgs::GenerationDeltaTime& _GenerationDeltaTime_out)
	{
		convert_toRos(_GenerationDeltaTime_in, _GenerationDeltaTime_out.value);
	}
	void convert_GenerationDeltaTimetoC(const etsi_its_cam_msgs::GenerationDeltaTime& _GenerationDeltaTime_in, GenerationDeltaTime_t& _GenerationDeltaTime_out)
	{
		memset(&_GenerationDeltaTime_out, 0, sizeof(GenerationDeltaTime_t));
		convert_toC(_GenerationDeltaTime_in.value, _GenerationDeltaTime_out);
	}
}