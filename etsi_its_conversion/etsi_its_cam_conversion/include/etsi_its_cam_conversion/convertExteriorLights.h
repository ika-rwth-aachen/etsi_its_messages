#pragma once

#include <etsi_its_cam_coding/ExteriorLights.h>
#include <etsi_its_cam_msgs/ExteriorLights.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::ExteriorLights convert_ExteriorLightstoRos(const ExteriorLights_t& _ExteriorLights_in)
	{
		etsi_its_cam_msgs::ExteriorLights ExteriorLights_out;
		convert_BIT_STRINGtoRos(_ExteriorLights_in, ExteriorLights_out.value);
		return ExteriorLights_out;
	}
	ExteriorLights_t convert_ExteriorLightstoC(const etsi_its_cam_msgs::ExteriorLights& _ExteriorLights_in)
	{
		ExteriorLights_t ExteriorLights_out;
		convert_BIT_STRINGtoC(_ExteriorLights_in.value, ExteriorLights_out);
		return ExteriorLights_out;
	}
}