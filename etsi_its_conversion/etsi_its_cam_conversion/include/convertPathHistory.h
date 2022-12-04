#pragma once

#include <PathHistory.h>
#include <etsi_its_cam_msgs/PathHistory.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PathHistory convert_PathHistorytoRos(const PathHistory_t& _PathHistory_in)
	{
		etsi_its_cam_msgs::PathHistory PathHistory_out;
		return PathHistory_out;
	}
}