#pragma once

#include <etsi_its_cam_coding/GenerationDeltaTime.h>
#include <etsi_its_cam_msgs/GenerationDeltaTime.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::GenerationDeltaTime convert_GenerationDeltaTimetoRos(const GenerationDeltaTime_t& _GenerationDeltaTime_in)
	{
		etsi_its_cam_msgs::GenerationDeltaTime GenerationDeltaTime_out;
		convert_toRos(_GenerationDeltaTime_in, GenerationDeltaTime_out.value);
		return GenerationDeltaTime_out;
	}
	GenerationDeltaTime_t convert_GenerationDeltaTimetoC(const etsi_its_cam_msgs::GenerationDeltaTime& _GenerationDeltaTime_in)
	{
		GenerationDeltaTime_t GenerationDeltaTime_out;
		memset(&GenerationDeltaTime_out, 0, sizeof(GenerationDeltaTime_t));
		convert_toC(_GenerationDeltaTime_in.value, GenerationDeltaTime_out);
		return GenerationDeltaTime_out;
	}
}