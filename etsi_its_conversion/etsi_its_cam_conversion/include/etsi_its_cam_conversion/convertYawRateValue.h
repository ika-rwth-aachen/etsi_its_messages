#pragma once

#include <etsi_its_cam_coding/YawRateValue.h>
#include <etsi_its_cam_msgs/YawRateValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::YawRateValue convert_YawRateValuetoRos(const YawRateValue_t& _YawRateValue_in)
	{
		etsi_its_cam_msgs::YawRateValue YawRateValue_out;
		convert_toRos(_YawRateValue_in, YawRateValue_out.value);
		return YawRateValue_out;
	}
}