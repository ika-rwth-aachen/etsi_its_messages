#pragma once

#include <RescueContainer.h>
#include <etsi_its_cam_msgs/RescueContainer.h>
#include <convertLightBarSirenInUse.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::RescueContainer convert_RescueContainertoRos(const RescueContainer_t& _RescueContainer_in)
	{
		etsi_its_cam_msgs::RescueContainer RescueContainer_out;
		RescueContainer_out.lightBarSirenInUse = convert_LightBarSirenInUsetoRos(_RescueContainer_in.lightBarSirenInUse);
		return RescueContainer_out;
	}
}