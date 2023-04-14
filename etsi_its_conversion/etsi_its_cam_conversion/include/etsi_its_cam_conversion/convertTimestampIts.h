#pragma once

#include <etsi_its_cam_coding/TimestampIts.h>
#include <etsi_its_cam_msgs/TimestampIts.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_TimestampItstoRos(const TimestampIts_t& _TimestampIts_in, etsi_its_cam_msgs::TimestampIts& _TimestampIts_out)
	{
		convert_toRos(_TimestampIts_in, _TimestampIts_out.value);
	}
	void convert_TimestampItstoC(const etsi_its_cam_msgs::TimestampIts& _TimestampIts_in, TimestampIts_t& _TimestampIts_out)
	{
		memset(&_TimestampIts_out, 0, sizeof(TimestampIts_t));
		convert_toC(_TimestampIts_in.value, _TimestampIts_out);
	}
}