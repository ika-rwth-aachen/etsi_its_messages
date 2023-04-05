#pragma once

#include <etsi_its_cam_coding/StationID.h>
#include <etsi_its_cam_msgs/StationID.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::StationID convert_StationIDtoRos(const StationID_t& _StationID_in)
	{
		etsi_its_cam_msgs::StationID StationID_out;
		convert_toRos(_StationID_in, StationID_out.value);
		return StationID_out;
	}
	StationID_t convert_StationIDtoC(const etsi_its_cam_msgs::StationID& _StationID_in)
	{
		StationID_t StationID_out;
		convert_toC(_StationID_in.value, StationID_out);
		return StationID_out;
	}
}