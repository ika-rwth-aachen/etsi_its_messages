#pragma once

#include <etsi_its_cam_coding/PathDeltaTime.h>
#include <etsi_its_cam_msgs/PathDeltaTime.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PathDeltaTime convert_PathDeltaTimetoRos(const PathDeltaTime_t& _PathDeltaTime_in)
	{
		etsi_its_cam_msgs::PathDeltaTime PathDeltaTime_out;
		convert_toRos(_PathDeltaTime_in, PathDeltaTime_out.value);
		return PathDeltaTime_out;
	}
	PathDeltaTime_t convert_PathDeltaTimetoC(const etsi_its_cam_msgs::PathDeltaTime& _PathDeltaTime_in)
	{
		PathDeltaTime_t PathDeltaTime_out;
		memset(&PathDeltaTime_out, 0, sizeof(PathDeltaTime_t));
		convert_toC(_PathDeltaTime_in.value, PathDeltaTime_out);
		return PathDeltaTime_out;
	}
}