#pragma once

#include <ExteriorLights.h>
#include <etsi_its_cam_msgs/ExteriorLights.h>
#include <primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::ExteriorLights convert_ExteriorLightstoRos(const ExteriorLights_t& _ExteriorLights_in)
	{
		etsi_its_cam_msgs::ExteriorLights ExteriorLights_out;
		convert_BIT_STRINGtoRos(_ExteriorLights_in, ExteriorLights_out.value);
		return ExteriorLights_out;
	}
}