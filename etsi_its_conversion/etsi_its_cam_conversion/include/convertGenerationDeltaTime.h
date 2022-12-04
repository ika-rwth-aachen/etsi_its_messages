#pragma once

#include <GenerationDeltaTime.h>
#include <etsi_its_cam_msgs/GenerationDeltaTime.h>
#include <primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::GenerationDeltaTime convert_GenerationDeltaTimetoRos(const GenerationDeltaTime_t& _GenerationDeltaTime_in)
	{
		etsi_its_cam_msgs::GenerationDeltaTime GenerationDeltaTime_out;
		convert_toRos(_GenerationDeltaTime_in, GenerationDeltaTime_out.value);
		return GenerationDeltaTime_out;
	}
}