#pragma once

#include <etsi_its_cam_coding/HeadingConfidence.h>
#include <etsi_its_cam_msgs/HeadingConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::HeadingConfidence convert_HeadingConfidencetoRos(const HeadingConfidence_t& _HeadingConfidence_in)
	{
		etsi_its_cam_msgs::HeadingConfidence HeadingConfidence_out;
		convert_toRos(_HeadingConfidence_in, HeadingConfidence_out.value);
		return HeadingConfidence_out;
	}
	HeadingConfidence_t convert_HeadingConfidencetoC(const etsi_its_cam_msgs::HeadingConfidence& _HeadingConfidence_in)
	{
		HeadingConfidence_t HeadingConfidence_out;
		convert_toC(_HeadingConfidence_in.value, HeadingConfidence_out);
		return HeadingConfidence_out;
	}
}