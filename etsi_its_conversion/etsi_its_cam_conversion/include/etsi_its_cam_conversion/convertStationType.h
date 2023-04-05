#pragma once

#include <etsi_its_cam_coding/StationType.h>
#include <etsi_its_cam_msgs/StationType.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::StationType convert_StationTypetoRos(const StationType_t& _StationType_in)
	{
		etsi_its_cam_msgs::StationType StationType_out;
		convert_toRos(_StationType_in, StationType_out.value);
		return StationType_out;
	}
	StationType_t convert_StationTypetoC(const etsi_its_cam_msgs::StationType& _StationType_in)
	{
		StationType_t StationType_out;
		memset(&StationType_out, 0, sizeof(StationType_t));
		convert_toC(_StationType_in.value, StationType_out);
		return StationType_out;
	}
}