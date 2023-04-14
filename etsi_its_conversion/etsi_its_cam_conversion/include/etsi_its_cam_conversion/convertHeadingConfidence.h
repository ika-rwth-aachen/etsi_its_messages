#pragma once

#include <etsi_its_cam_coding/HeadingConfidence.h>
#include <etsi_its_cam_msgs/HeadingConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_HeadingConfidencetoRos(const HeadingConfidence_t& _HeadingConfidence_in, etsi_its_cam_msgs::HeadingConfidence& _HeadingConfidence_out)
	{
		convert_toRos(_HeadingConfidence_in, _HeadingConfidence_out.value);
	}
	void convert_HeadingConfidencetoC(const etsi_its_cam_msgs::HeadingConfidence& _HeadingConfidence_in, HeadingConfidence_t& _HeadingConfidence_out)
	{
		memset(&_HeadingConfidence_out, 0, sizeof(HeadingConfidence_t));
		convert_toC(_HeadingConfidence_in.value, _HeadingConfidence_out);
	}
}