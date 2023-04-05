#pragma once

#include <etsi_its_cam_coding/HighFrequencyContainer.h>
#include <etsi_its_cam_msgs/HighFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertBasicVehicleContainerHighFrequency.h>
#include <etsi_its_cam_conversion/convertRSUContainerHighFrequency.h>

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
	HighFrequencyContainer_t convert_HighFrequencyContainertoC(const etsi_its_cam_msgs::HighFrequencyContainer& _HighFrequencyContainer_in)
	{
		HighFrequencyContainer_t HighFrequencyContainer_out;
		if(_HighFrequencyContainer_in.choice == etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY)
		{
			HighFrequencyContainer_out.choice.basicVehicleContainerHighFrequency = convert_BasicVehicleContainerHighFrequencytoC(_HighFrequencyContainer_in.basicVehicleContainerHighFrequency);
			HighFrequencyContainer_out.present = HighFrequencyContainer_PR::HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
		}
		if(_HighFrequencyContainer_in.choice == etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY)
		{
			HighFrequencyContainer_out.choice.rsuContainerHighFrequency = convert_RSUContainerHighFrequencytoC(_HighFrequencyContainer_in.rsuContainerHighFrequency);
			HighFrequencyContainer_out.present = HighFrequencyContainer_PR::HighFrequencyContainer_PR_rsuContainerHighFrequency;
		}
		return HighFrequencyContainer_out;
	}
}