#pragma once

#include <etsi_its_cam_coding/TimestampIts.h>
#include <etsi_its_cam_msgs/TimestampIts.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::TimestampIts convert_TimestampItstoRos(const TimestampIts_t& _TimestampIts_in)
	{
		etsi_its_cam_msgs::TimestampIts TimestampIts_out;
		convert_toRos(_TimestampIts_in, TimestampIts_out.value);
		return TimestampIts_out;
	}
	TimestampIts_t convert_TimestampItstoC(const etsi_its_cam_msgs::TimestampIts& _TimestampIts_in)
	{
		TimestampIts_t TimestampIts_out;
		convert_toC(_TimestampIts_in.value, TimestampIts_out);
		return TimestampIts_out;
	}
}