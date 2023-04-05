#pragma once

#include <etsi_its_cam_coding/RescueContainer.h>
#include <etsi_its_cam_msgs/RescueContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::RescueContainer convert_RescueContainertoRos(const RescueContainer_t& _RescueContainer_in)
	{
		etsi_its_cam_msgs::RescueContainer RescueContainer_out;
		RescueContainer_out.lightBarSirenInUse = convert_LightBarSirenInUsetoRos(_RescueContainer_in.lightBarSirenInUse);
		return RescueContainer_out;
	}
	RescueContainer_t convert_RescueContainertoC(const etsi_its_cam_msgs::RescueContainer& _RescueContainer_in)
	{
		RescueContainer_t RescueContainer_out;
		memset(&RescueContainer_out, 0, sizeof(RescueContainer_t));
		RescueContainer_out.lightBarSirenInUse = convert_LightBarSirenInUsetoC(_RescueContainer_in.lightBarSirenInUse);
		return RescueContainer_out;
	}
}