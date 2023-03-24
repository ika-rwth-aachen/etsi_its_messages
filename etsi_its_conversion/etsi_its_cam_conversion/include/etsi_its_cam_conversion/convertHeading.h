#pragma once

#include <etsi_its_cam_coding/Heading.h>
#include <etsi_its_cam_msgs/Heading.h>
#include <etsi_its_cam_conversion/convertHeadingValue.h>
#include <etsi_its_cam_conversion/convertHeadingConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::Heading convert_HeadingtoRos(const Heading_t& _Heading_in)
	{
		etsi_its_cam_msgs::Heading Heading_out;
		Heading_out.headingValue = convert_HeadingValuetoRos(_Heading_in.headingValue);
		Heading_out.headingConfidence = convert_HeadingConfidencetoRos(_Heading_in.headingConfidence);
		return Heading_out;
	}
}