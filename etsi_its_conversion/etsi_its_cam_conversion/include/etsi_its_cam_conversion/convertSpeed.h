#pragma once

#include <etsi_its_cam_coding/Speed.h>
#include <etsi_its_cam_msgs/Speed.h>
#include <etsi_its_cam_conversion/convertSpeedValue.h>
#include <etsi_its_cam_conversion/convertSpeedConfidence.h>

namespace etsi_its_cam_conversion
{
	void convert_SpeedtoRos(const Speed_t& _Speed_in, etsi_its_cam_msgs::Speed& _Speed_out)
	{
		convert_SpeedValuetoRos(_Speed_in.speedValue, _Speed_out.speedValue);
		convert_SpeedConfidencetoRos(_Speed_in.speedConfidence, _Speed_out.speedConfidence);
	}
	void convert_SpeedtoC(const etsi_its_cam_msgs::Speed& _Speed_in, Speed_t& _Speed_out)
	{
		memset(&_Speed_out, 0, sizeof(Speed_t));
		convert_SpeedValuetoC(_Speed_in.speedValue, _Speed_out.speedValue);
		convert_SpeedConfidencetoC(_Speed_in.speedConfidence, _Speed_out.speedConfidence);
	}
}