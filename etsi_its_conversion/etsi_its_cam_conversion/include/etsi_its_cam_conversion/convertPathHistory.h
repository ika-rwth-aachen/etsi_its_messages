#pragma once

#include <etsi_its_cam_coding/PathHistory.h>
#include <etsi_its_cam_msgs/PathHistory.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PathHistory convert_PathHistorytoRos(const PathHistory_t& _PathHistory_in)
	{
		etsi_its_cam_msgs::PathHistory PathHistory_out;
		return PathHistory_out;
	}
	PathHistory_t convert_PathHistorytoC(const etsi_its_cam_msgs::PathHistory& _PathHistory_in)
	{
		PathHistory_t PathHistory_out;
		return PathHistory_out;
	}
}