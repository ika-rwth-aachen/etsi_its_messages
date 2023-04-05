#pragma once

#include <etsi_its_cam_coding/SpecialTransportContainer.h>
#include <etsi_its_cam_msgs/SpecialTransportContainer.h>
#include <etsi_its_cam_conversion/convertSpecialTransportType.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SpecialTransportContainer convert_SpecialTransportContainertoRos(const SpecialTransportContainer_t& _SpecialTransportContainer_in)
	{
		etsi_its_cam_msgs::SpecialTransportContainer SpecialTransportContainer_out;
		SpecialTransportContainer_out.specialTransportType = convert_SpecialTransportTypetoRos(_SpecialTransportContainer_in.specialTransportType);
		SpecialTransportContainer_out.lightBarSirenInUse = convert_LightBarSirenInUsetoRos(_SpecialTransportContainer_in.lightBarSirenInUse);
		return SpecialTransportContainer_out;
	}
	SpecialTransportContainer_t convert_SpecialTransportContainertoC(const etsi_its_cam_msgs::SpecialTransportContainer& _SpecialTransportContainer_in)
	{
		SpecialTransportContainer_t SpecialTransportContainer_out;
		memset(&SpecialTransportContainer_out, 0, sizeof(SpecialTransportContainer_t));
		SpecialTransportContainer_out.specialTransportType = convert_SpecialTransportTypetoC(_SpecialTransportContainer_in.specialTransportType);
		SpecialTransportContainer_out.lightBarSirenInUse = convert_LightBarSirenInUsetoC(_SpecialTransportContainer_in.lightBarSirenInUse);
		return SpecialTransportContainer_out;
	}
}