#pragma once

#include <Speed.h>
#include <etsi_its_cam_msgs/Speed.h>
#include <convertSpeedValue.h>
#include <convertSpeedConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::Speed convert_SpeedtoRos(const Speed_t& _Speed_in)
	{
		etsi_its_cam_msgs::Speed Speed_out;
		Speed_out.speedValue = convert_SpeedValuetoRos(_Speed_in.speedValue);
		Speed_out.speedConfidence = convert_SpeedConfidencetoRos(_Speed_in.speedConfidence);
		return Speed_out;
	}
}