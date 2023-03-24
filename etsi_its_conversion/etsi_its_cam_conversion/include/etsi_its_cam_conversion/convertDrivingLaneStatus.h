#pragma once

#include <etsi_its_cam_coding/DrivingLaneStatus.h>
#include <etsi_its_cam_msgs/DrivingLaneStatus.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DrivingLaneStatus convert_DrivingLaneStatustoRos(const DrivingLaneStatus_t& _DrivingLaneStatus_in)
	{
		etsi_its_cam_msgs::DrivingLaneStatus DrivingLaneStatus_out;
		convert_BIT_STRINGtoRos(_DrivingLaneStatus_in, DrivingLaneStatus_out.value);
		return DrivingLaneStatus_out;
	}
}