#pragma once

#include <etsi_its_cam_coding/DriveDirection.h>
#include <etsi_its_cam_msgs/DriveDirection.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DriveDirection convert_DriveDirectiontoRos(const DriveDirection_t& _DriveDirection_in)
	{
		etsi_its_cam_msgs::DriveDirection DriveDirection_out;
		DriveDirection_out.value = _DriveDirection_in;
		return DriveDirection_out;
	}
	DriveDirection_t convert_DriveDirectiontoC(const etsi_its_cam_msgs::DriveDirection& _DriveDirection_in)
	{
		DriveDirection_t DriveDirection_out;
		memset(&DriveDirection_out, 0, sizeof(DriveDirection_t));
		DriveDirection_out = _DriveDirection_in.value;
		return DriveDirection_out;
	}
}