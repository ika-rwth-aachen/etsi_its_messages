#pragma once

#include <etsi_its_cam_coding/CamParameters.h>
#include <etsi_its_cam_msgs/CamParameters.h>
#include <etsi_its_cam_conversion/convertBasicContainer.h>
#include <etsi_its_cam_conversion/convertHighFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertLowFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertSpecialVehicleContainer.h>

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
	CamParameters_t convert_CamParameterstoC(const etsi_its_cam_msgs::CamParameters& _CamParameters_in)
	{
		CamParameters_t CamParameters_out;
		memset(&CamParameters_out, 0, sizeof(CamParameters_t));
		CamParameters_out.basicContainer = convert_BasicContainertoC(_CamParameters_in.basicContainer);
		CamParameters_out.highFrequencyContainer = convert_HighFrequencyContainertoC(_CamParameters_in.highFrequencyContainer);
		if(_CamParameters_in.lowFrequencyContainer_isPresent)
		{
			auto lowFrequencyContainer = convert_LowFrequencyContainertoC(_CamParameters_in.lowFrequencyContainer);
			CamParameters_out.lowFrequencyContainer = new LowFrequencyContainer_t(lowFrequencyContainer);
		}
		if(_CamParameters_in.specialVehicleContainer_isPresent)
		{
			auto specialVehicleContainer = convert_SpecialVehicleContainertoC(_CamParameters_in.specialVehicleContainer);
			CamParameters_out.specialVehicleContainer = new SpecialVehicleContainer_t(specialVehicleContainer);
		}
		return CamParameters_out;
	}
}