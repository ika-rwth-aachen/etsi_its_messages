#pragma once

#include <TimestampIts.h>
#include <etsi_its_cam_msgs/TimestampIts.h>
#include <convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::TimestampIts convert_TimestampItstoRos(const TimestampIts_t& _TimestampIts_in)
	{
		etsi_its_cam_msgs::TimestampIts TimestampIts_out;
		convert_toRos(_TimestampIts_in, TimestampIts_out.value);
		return TimestampIts_out;
	}
}