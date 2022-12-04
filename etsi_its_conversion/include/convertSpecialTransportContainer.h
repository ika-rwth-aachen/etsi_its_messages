#pragma once

#include <SpecialTransportContainer.h>
#include <etsi_its_cam_msgs/SpecialTransportContainer.h>
#include <convertSpecialTransportType.h>
#include <convertLightBarSirenInUse.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SpecialTransportContainer convert_SpecialTransportContainertoRos(const SpecialTransportContainer_t& _SpecialTransportContainer_in)
	{
		etsi_its_cam_msgs::SpecialTransportContainer SpecialTransportContainer_out;
		SpecialTransportContainer_out.specialTransportType = convert_SpecialTransportTypetoRos(_SpecialTransportContainer_in.specialTransportType);
		SpecialTransportContainer_out.lightBarSirenInUse = convert_LightBarSirenInUsetoRos(_SpecialTransportContainer_in.lightBarSirenInUse);
		return SpecialTransportContainer_out;
	}
}