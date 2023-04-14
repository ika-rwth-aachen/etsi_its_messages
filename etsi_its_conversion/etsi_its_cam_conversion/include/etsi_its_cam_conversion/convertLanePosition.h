#pragma once

#include <etsi_its_cam_coding/LanePosition.h>
#include <etsi_its_cam_msgs/LanePosition.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_LanePositiontoRos(const LanePosition_t& _LanePosition_in, etsi_its_cam_msgs::LanePosition& _LanePosition_out)
	{
		convert_toRos(_LanePosition_in, _LanePosition_out.value);
	}
	void convert_LanePositiontoC(const etsi_its_cam_msgs::LanePosition& _LanePosition_in, LanePosition_t& _LanePosition_out)
	{
		memset(&_LanePosition_out, 0, sizeof(LanePosition_t));
		convert_toC(_LanePosition_in.value, _LanePosition_out);
	}
}