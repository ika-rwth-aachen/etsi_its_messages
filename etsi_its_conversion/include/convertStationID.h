#pragma once

#include <StationID.h>
#include <etsi_its_cam_msgs/StationID.h>
#include <convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::StationID convert_StationIDtoRos(const StationID_t& _StationID_in)
	{
		etsi_its_cam_msgs::StationID StationID_out;
		convert_toRos(_StationID_in, StationID_out.value);
		return StationID_out;
	}
}