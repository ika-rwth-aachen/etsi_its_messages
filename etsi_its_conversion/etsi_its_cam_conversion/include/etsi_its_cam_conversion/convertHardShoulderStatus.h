#pragma once

#include <etsi_its_cam_coding/HardShoulderStatus.h>
#include <etsi_its_cam_msgs/HardShoulderStatus.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::HardShoulderStatus convert_HardShoulderStatustoRos(const HardShoulderStatus_t& _HardShoulderStatus_in)
	{
		etsi_its_cam_msgs::HardShoulderStatus HardShoulderStatus_out;
		HardShoulderStatus_out.value = _HardShoulderStatus_in;
		return HardShoulderStatus_out;
	}
	HardShoulderStatus_t convert_HardShoulderStatustoC(const etsi_its_cam_msgs::HardShoulderStatus& _HardShoulderStatus_in)
	{
		HardShoulderStatus_t HardShoulderStatus_out;
		HardShoulderStatus_out = _HardShoulderStatus_in.value;
		return HardShoulderStatus_out;
	}
}