#pragma once

#include <etsi_its_cam_coding/ExteriorLights.h>
#include <etsi_its_cam_msgs/ExteriorLights.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion
{
	void convert_ExteriorLightstoRos(const ExteriorLights_t& _ExteriorLights_in, etsi_its_cam_msgs::ExteriorLights& _ExteriorLights_out)
	{
		convert_BIT_STRINGtoRos(_ExteriorLights_in, _ExteriorLights_out.value);
	}
	void convert_ExteriorLightstoC(const etsi_its_cam_msgs::ExteriorLights& _ExteriorLights_in, ExteriorLights_t& _ExteriorLights_out)
	{
		memset(&_ExteriorLights_out, 0, sizeof(ExteriorLights_t));
		convert_BIT_STRINGtoC(_ExteriorLights_in.value, _ExteriorLights_out);
	}
}