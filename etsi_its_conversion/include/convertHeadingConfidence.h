#pragma once

#include <HeadingConfidence.h>
#include <etsi_its_cam_msgs/HeadingConfidence.h>
#include <convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::HeadingConfidence convert_HeadingConfidencetoRos(const HeadingConfidence_t& _HeadingConfidence_in)
	{
		etsi_its_cam_msgs::HeadingConfidence HeadingConfidence_out;
		convert_toRos(_HeadingConfidence_in, HeadingConfidence_out.value);
		return HeadingConfidence_out;
	}
}