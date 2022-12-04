#pragma once

#include <DriveDirection.h>
#include <etsi_its_cam_msgs/DriveDirection.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DriveDirection convert_DriveDirectiontoRos(const DriveDirection_t& _DriveDirection_in)
	{
		etsi_its_cam_msgs::DriveDirection DriveDirection_out;
		DriveDirection_out.value = _DriveDirection_in;
		return DriveDirection_out;
	}
}