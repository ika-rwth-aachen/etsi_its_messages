#pragma once

#include <etsi_its_cam_coding/Speed.h>
#include <etsi_its_cam_msgs/Speed.h>
#include <etsi_its_cam_conversion/convertSpeedValue.h>
#include <etsi_its_cam_conversion/convertSpeedConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::Speed convert_SpeedtoRos(const Speed_t& _Speed_in)
	{
		etsi_its_cam_msgs::Speed Speed_out;
		Speed_out.speedValue = convert_SpeedValuetoRos(_Speed_in.speedValue);
		Speed_out.speedConfidence = convert_SpeedConfidencetoRos(_Speed_in.speedConfidence);
		return Speed_out;
	}
	Speed_t convert_SpeedtoC(const etsi_its_cam_msgs::Speed& _Speed_in)
	{
		Speed_t Speed_out;
		memset(&Speed_out, 0, sizeof(Speed_t));
		Speed_out.speedValue = convert_SpeedValuetoC(_Speed_in.speedValue);
		Speed_out.speedConfidence = convert_SpeedConfidencetoC(_Speed_in.speedConfidence);
		return Speed_out;
	}
}