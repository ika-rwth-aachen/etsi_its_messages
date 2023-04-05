#pragma once

#include <etsi_its_cam_coding/VerticalAcceleration.h>
#include <etsi_its_cam_msgs/VerticalAcceleration.h>
#include <etsi_its_cam_conversion/convertVerticalAccelerationValue.h>
#include <etsi_its_cam_conversion/convertAccelerationConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::VerticalAcceleration convert_VerticalAccelerationtoRos(const VerticalAcceleration_t& _VerticalAcceleration_in)
	{
		etsi_its_cam_msgs::VerticalAcceleration VerticalAcceleration_out;
		VerticalAcceleration_out.verticalAccelerationValue = convert_VerticalAccelerationValuetoRos(_VerticalAcceleration_in.verticalAccelerationValue);
		VerticalAcceleration_out.verticalAccelerationConfidence = convert_AccelerationConfidencetoRos(_VerticalAcceleration_in.verticalAccelerationConfidence);
		return VerticalAcceleration_out;
	}
	VerticalAcceleration_t convert_VerticalAccelerationtoC(const etsi_its_cam_msgs::VerticalAcceleration& _VerticalAcceleration_in)
	{
		VerticalAcceleration_t VerticalAcceleration_out;
		VerticalAcceleration_out.verticalAccelerationValue = convert_VerticalAccelerationValuetoC(_VerticalAcceleration_in.verticalAccelerationValue);
		VerticalAcceleration_out.verticalAccelerationConfidence = convert_AccelerationConfidencetoC(_VerticalAcceleration_in.verticalAccelerationConfidence);
		return VerticalAcceleration_out;
	}
}