#pragma once

#include <CamParameters.h>
#include <etsi_its_cam_msgs/CamParameters.h>
#include <convertBasicContainer.h>
#include <convertHighFrequencyContainer.h>
#include <convertLowFrequencyContainer.h>
#include <convertSpecialVehicleContainer.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::CamParameters convert_CamParameterstoRos(const CamParameters_t& _CamParameters_in)
	{
		etsi_its_cam_msgs::CamParameters CamParameters_out;
		CamParameters_out.basicContainer = convert_BasicContainertoRos(_CamParameters_in.basicContainer);
		CamParameters_out.highFrequencyContainer = convert_HighFrequencyContainertoRos(_CamParameters_in.highFrequencyContainer);
		if(_CamParameters_in.lowFrequencyContainer)
		{
			CamParameters_out.lowFrequencyContainer = convert_LowFrequencyContainertoRos(*_CamParameters_in.lowFrequencyContainer);
			CamParameters_out.lowFrequencyContainer_isPresent = true;
		}
		if(_CamParameters_in.specialVehicleContainer)
		{
			CamParameters_out.specialVehicleContainer = convert_SpecialVehicleContainertoRos(*_CamParameters_in.specialVehicleContainer);
			CamParameters_out.specialVehicleContainer_isPresent = true;
		}
		return CamParameters_out;
	}
}