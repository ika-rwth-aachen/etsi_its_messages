#pragma once

#include <LightBarSirenInUse.h>
#include <etsi_its_cam_msgs/LightBarSirenInUse.h>
#include <convertBIT_STRING.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::LightBarSirenInUse convert_LightBarSirenInUsetoRos(const LightBarSirenInUse_t& _LightBarSirenInUse_in)
	{
		etsi_its_cam_msgs::LightBarSirenInUse LightBarSirenInUse_out;
		convert_BIT_STRINGtoRos(_LightBarSirenInUse_in, LightBarSirenInUse_out.value);
		return LightBarSirenInUse_out;
	}
}