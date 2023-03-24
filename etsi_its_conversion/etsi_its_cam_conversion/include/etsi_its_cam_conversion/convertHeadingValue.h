#pragma once

#include <etsi_its_cam_coding/HeadingValue.h>
#include <etsi_its_cam_msgs/HeadingValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::HeadingValue convert_HeadingValuetoRos(const HeadingValue_t& _HeadingValue_in)
	{
		etsi_its_cam_msgs::HeadingValue HeadingValue_out;
		convert_toRos(_HeadingValue_in, HeadingValue_out.value);
		return HeadingValue_out;
	}
}