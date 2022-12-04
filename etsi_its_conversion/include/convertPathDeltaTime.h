#pragma once

#include <PathDeltaTime.h>
#include <etsi_its_cam_msgs/PathDeltaTime.h>
#include <convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PathDeltaTime convert_PathDeltaTimetoRos(const PathDeltaTime_t& _PathDeltaTime_in)
	{
		etsi_its_cam_msgs::PathDeltaTime PathDeltaTime_out;
		convert_toRos(_PathDeltaTime_in, PathDeltaTime_out.value);
		return PathDeltaTime_out;
	}
}