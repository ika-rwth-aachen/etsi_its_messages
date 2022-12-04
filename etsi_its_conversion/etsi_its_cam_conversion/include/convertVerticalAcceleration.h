#pragma once

#include <VerticalAcceleration.h>
#include <etsi_its_cam_msgs/VerticalAcceleration.h>
#include <convertVerticalAccelerationValue.h>
#include <convertAccelerationConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::VerticalAcceleration convert_VerticalAccelerationtoRos(const VerticalAcceleration_t& _VerticalAcceleration_in)
	{
		etsi_its_cam_msgs::VerticalAcceleration VerticalAcceleration_out;
		VerticalAcceleration_out.verticalAccelerationValue = convert_VerticalAccelerationValuetoRos(_VerticalAcceleration_in.verticalAccelerationValue);
		VerticalAcceleration_out.verticalAccelerationConfidence = convert_AccelerationConfidencetoRos(_VerticalAcceleration_in.verticalAccelerationConfidence);
		return VerticalAcceleration_out;
	}
}