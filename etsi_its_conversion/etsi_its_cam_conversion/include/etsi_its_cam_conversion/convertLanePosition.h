#pragma once

#include <etsi_its_cam_coding/LanePosition.h>
#include <etsi_its_cam_msgs/LanePosition.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::LanePosition convert_LanePositiontoRos(const LanePosition_t& _LanePosition_in)
	{
		etsi_its_cam_msgs::LanePosition LanePosition_out;
		convert_toRos(_LanePosition_in, LanePosition_out.value);
		return LanePosition_out;
	}
	LanePosition_t convert_LanePositiontoC(const etsi_its_cam_msgs::LanePosition& _LanePosition_in)
	{
		LanePosition_t LanePosition_out;
		memset(&LanePosition_out, 0, sizeof(LanePosition_t));
		convert_toC(_LanePosition_in.value, LanePosition_out);
		return LanePosition_out;
	}
}