#pragma once

#include <etsi_its_cam_coding/PathHistory.h>
#include <etsi_its_cam_msgs/PathHistory.h>

namespace etsi_its_cam_conversion
{
	void convert_PathHistorytoRos(const PathHistory_t& _PathHistory_in, etsi_its_cam_msgs::PathHistory& _PathHistory_out)
	{
	}
	void convert_PathHistorytoC(const etsi_its_cam_msgs::PathHistory& _PathHistory_in, PathHistory_t& _PathHistory_out)
	{
		memset(&_PathHistory_out, 0, sizeof(PathHistory_t));
	}
}