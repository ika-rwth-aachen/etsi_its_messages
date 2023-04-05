#pragma once

#include <etsi_its_cam_coding/LightBarSirenInUse.h>
#include <etsi_its_cam_msgs/LightBarSirenInUse.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::LightBarSirenInUse convert_LightBarSirenInUsetoRos(const LightBarSirenInUse_t& _LightBarSirenInUse_in)
	{
		etsi_its_cam_msgs::LightBarSirenInUse LightBarSirenInUse_out;
		convert_BIT_STRINGtoRos(_LightBarSirenInUse_in, LightBarSirenInUse_out.value);
		return LightBarSirenInUse_out;
	}
	LightBarSirenInUse_t convert_LightBarSirenInUsetoC(const etsi_its_cam_msgs::LightBarSirenInUse& _LightBarSirenInUse_in)
	{
		LightBarSirenInUse_t LightBarSirenInUse_out;
		convert_BIT_STRINGtoC(_LightBarSirenInUse_in.value, LightBarSirenInUse_out);
		return LightBarSirenInUse_out;
	}
}