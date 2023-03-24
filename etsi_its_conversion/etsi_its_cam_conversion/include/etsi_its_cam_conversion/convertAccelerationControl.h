#pragma once

#include <etsi_its_cam_coding/AccelerationControl.h>
#include <etsi_its_cam_msgs/AccelerationControl.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::AccelerationControl convert_AccelerationControltoRos(const AccelerationControl_t& _AccelerationControl_in)
	{
		etsi_its_cam_msgs::AccelerationControl AccelerationControl_out;
		convert_BIT_STRINGtoRos(_AccelerationControl_in, AccelerationControl_out.value);
		return AccelerationControl_out;
	}
}