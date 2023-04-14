#pragma once

#include <etsi_its_cam_coding/PathDeltaTime.h>
#include <etsi_its_cam_msgs/PathDeltaTime.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_PathDeltaTimetoRos(const PathDeltaTime_t& _PathDeltaTime_in, etsi_its_cam_msgs::PathDeltaTime& _PathDeltaTime_out)
	{
		convert_toRos(_PathDeltaTime_in, _PathDeltaTime_out.value);
	}
	void convert_PathDeltaTimetoC(const etsi_its_cam_msgs::PathDeltaTime& _PathDeltaTime_in, PathDeltaTime_t& _PathDeltaTime_out)
	{
		memset(&_PathDeltaTime_out, 0, sizeof(PathDeltaTime_t));
		convert_toC(_PathDeltaTime_in.value, _PathDeltaTime_out);
	}
}