#pragma once

#include <etsi_its_cam_coding/HighFrequencyContainer.h>
#include <etsi_its_cam_msgs/HighFrequencyContainer.h>
#include <etsi_its_cam_conversion/convertBasicVehicleContainerHighFrequency.h>
#include <etsi_its_cam_conversion/convertRSUContainerHighFrequency.h>

namespace etsi_its_cam_conversion
{
	void convert_HighFrequencyContainertoRos(const HighFrequencyContainer_t& _HighFrequencyContainer_in, etsi_its_cam_msgs::HighFrequencyContainer& _HighFrequencyContainer_out)
	{
		if(_HighFrequencyContainer_in.present == HighFrequencyContainer_PR::HighFrequencyContainer_PR_basicVehicleContainerHighFrequency)
		{
			convert_BasicVehicleContainerHighFrequencytoRos(_HighFrequencyContainer_in.choice.basicVehicleContainerHighFrequency, _HighFrequencyContainer_out.basicVehicleContainerHighFrequency);
			_HighFrequencyContainer_out.choice = etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY;
		}
		if(_HighFrequencyContainer_in.present == HighFrequencyContainer_PR::HighFrequencyContainer_PR_rsuContainerHighFrequency)
		{
			convert_RSUContainerHighFrequencytoRos(_HighFrequencyContainer_in.choice.rsuContainerHighFrequency, _HighFrequencyContainer_out.rsuContainerHighFrequency);
			_HighFrequencyContainer_out.choice = etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY;
		}
	}
	void convert_HighFrequencyContainertoC(const etsi_its_cam_msgs::HighFrequencyContainer& _HighFrequencyContainer_in, HighFrequencyContainer_t& _HighFrequencyContainer_out)
	{
		memset(&_HighFrequencyContainer_out, 0, sizeof(HighFrequencyContainer_t));
		if(_HighFrequencyContainer_in.choice == etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY)
		{
			convert_BasicVehicleContainerHighFrequencytoC(_HighFrequencyContainer_in.basicVehicleContainerHighFrequency, _HighFrequencyContainer_out.choice.basicVehicleContainerHighFrequency);
			_HighFrequencyContainer_out.present = HighFrequencyContainer_PR::HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
		}
		if(_HighFrequencyContainer_in.choice == etsi_its_cam_msgs::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY)
		{
			convert_RSUContainerHighFrequencytoC(_HighFrequencyContainer_in.rsuContainerHighFrequency, _HighFrequencyContainer_out.choice.rsuContainerHighFrequency);
			_HighFrequencyContainer_out.present = HighFrequencyContainer_PR::HighFrequencyContainer_PR_rsuContainerHighFrequency;
		}
	}
}