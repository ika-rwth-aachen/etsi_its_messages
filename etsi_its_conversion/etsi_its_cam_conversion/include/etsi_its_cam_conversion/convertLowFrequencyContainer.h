#pragma once

#include <etsi_its_cam_coding/LowFrequencyContainer.h>
#include <etsi_its_cam_msgs/LowFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertBasicVehicleContainerLowFrequency.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::LowFrequencyContainer convert_LowFrequencyContainertoRos(const LowFrequencyContainer_t& _LowFrequencyContainer_in)
	{
		etsi_its_cam_msgs::LowFrequencyContainer LowFrequencyContainer_out;
		if(_LowFrequencyContainer_in.present == LowFrequencyContainer_PR::LowFrequencyContainer_PR_basicVehicleContainerLowFrequency)
		{
			LowFrequencyContainer_out.basicVehicleContainerLowFrequency = convert_BasicVehicleContainerLowFrequencytoRos(_LowFrequencyContainer_in.choice.basicVehicleContainerLowFrequency);
			LowFrequencyContainer_out.choice = etsi_its_cam_msgs::LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY;
		}
		return LowFrequencyContainer_out;
	}
}