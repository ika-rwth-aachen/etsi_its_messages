#pragma once

#include <LanePosition.h>
#include <etsi_its_cam_msgs/LanePosition.h>
#include <primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::LanePosition convert_LanePositiontoRos(const LanePosition_t& _LanePosition_in)
	{
		etsi_its_cam_msgs::LanePosition LanePosition_out;
		convert_toRos(_LanePosition_in, LanePosition_out.value);
		return LanePosition_out;
	}
}