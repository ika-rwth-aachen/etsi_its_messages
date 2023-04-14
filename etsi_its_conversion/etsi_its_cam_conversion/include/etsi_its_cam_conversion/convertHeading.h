#pragma once

#include <etsi_its_cam_coding/Heading.h>
#include <etsi_its_cam_msgs/Heading.h>
#include <etsi_its_cam_conversion/convertHeadingValue.h>
#include <etsi_its_cam_conversion/convertHeadingConfidence.h>

namespace etsi_its_cam_conversion
{
	void convert_HeadingtoRos(const Heading_t& _Heading_in, etsi_its_cam_msgs::Heading& _Heading_out)
	{
		convert_HeadingValuetoRos(_Heading_in.headingValue, _Heading_out.headingValue);
		convert_HeadingConfidencetoRos(_Heading_in.headingConfidence, _Heading_out.headingConfidence);
	}
	void convert_HeadingtoC(const etsi_its_cam_msgs::Heading& _Heading_in, Heading_t& _Heading_out)
	{
		memset(&_Heading_out, 0, sizeof(Heading_t));
		convert_HeadingValuetoC(_Heading_in.headingValue, _Heading_out.headingValue);
		convert_HeadingConfidencetoC(_Heading_in.headingConfidence, _Heading_out.headingConfidence);
	}
}