#pragma once

#include <HighFrequencyContainer.h>
#include <etsi_its_cam_msgs/HighFrequencyContainer.h>
#include <convertBasicVehicleContainerHighFrequency.h>
#include <convertRSUContainerHighFrequency.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::HighFrequencyContainer convert_HighFrequencyContainertoRos(const HighFrequencyContainer_t& _HighFrequencyContainer_in)
	{
		etsi_its_cam_msgs::HighFrequencyContainer HighFrequencyContainer_out;
		if(_HighFrequencyContainer_in.present == HighFrequencyContainer_PR::HighFrequencyContainer_PR_basicVehicleContainerHighFrequency)
		{
			HighFrequencyContainer_out.basicVehicleContainerHighFrequency = convert_BasicVehicleContainerHighFrequencytoRos(_HighFrequencyContainer_in.choice.basicVehicleContainerHighFrequency);
			HighFrequencyContainer_out.choice = etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;
		}
		if(_HighFrequencyContainer_in.present == HighFrequencyContainer_PR::HighFrequencyContainer_PR_rsuContainerHighFrequency)
		{
			HighFrequencyContainer_out.rsuContainerHighFrequency = convert_RSUContainerHighFrequencytoRos(_HighFrequencyContainer_in.choice.rsuContainerHighFrequency);
			HighFrequencyContainer_out.choice = etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY;
		}
		return HighFrequencyContainer_out;
	}
}