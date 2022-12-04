#pragma once

#include <LateralAcceleration.h>
#include <etsi_its_cam_msgs/LateralAcceleration.h>
#include <convertLateralAccelerationValue.h>
#include <convertAccelerationConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::LateralAcceleration convert_LateralAccelerationtoRos(const LateralAcceleration_t& _LateralAcceleration_in)
	{
		etsi_its_cam_msgs::LateralAcceleration LateralAcceleration_out;
		LateralAcceleration_out.lateralAccelerationValue = convert_LateralAccelerationValuetoRos(_LateralAcceleration_in.lateralAccelerationValue);
		LateralAcceleration_out.lateralAccelerationConfidence = convert_AccelerationConfidencetoRos(_LateralAcceleration_in.lateralAccelerationConfidence);
		return LateralAcceleration_out;
	}
}