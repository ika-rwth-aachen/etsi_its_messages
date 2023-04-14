#pragma once

#include <etsi_its_cam_coding/AccelerationControl.h>
#include <etsi_its_cam_msgs/AccelerationControl.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion
{
	void convert_AccelerationControltoRos(const AccelerationControl_t& _AccelerationControl_in, etsi_its_cam_msgs::AccelerationControl& _AccelerationControl_out)
	{
		convert_BIT_STRINGtoRos(_AccelerationControl_in, _AccelerationControl_out.value);
	}
	void convert_AccelerationControltoC(const etsi_its_cam_msgs::AccelerationControl& _AccelerationControl_in, AccelerationControl_t& _AccelerationControl_out)
	{
		memset(&_AccelerationControl_out, 0, sizeof(AccelerationControl_t));
		convert_BIT_STRINGtoC(_AccelerationControl_in.value, _AccelerationControl_out);
	}
}