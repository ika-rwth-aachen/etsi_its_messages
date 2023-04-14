#pragma once

#include <etsi_its_cam_coding/HardShoulderStatus.h>
#include <etsi_its_cam_msgs/HardShoulderStatus.h>

namespace etsi_its_cam_conversion
{
	void convert_HardShoulderStatustoRos(const HardShoulderStatus_t& _HardShoulderStatus_in, etsi_its_cam_msgs::HardShoulderStatus& _HardShoulderStatus_out)
	{
		_HardShoulderStatus_out.value = _HardShoulderStatus_in;
	}
	void convert_HardShoulderStatustoC(const etsi_its_cam_msgs::HardShoulderStatus& _HardShoulderStatus_in, HardShoulderStatus_t& _HardShoulderStatus_out)
	{
		memset(&_HardShoulderStatus_out, 0, sizeof(HardShoulderStatus_t));
		_HardShoulderStatus_out = _HardShoulderStatus_in.value;
	}
}