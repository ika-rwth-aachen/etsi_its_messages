#pragma once

#include <etsi_its_cam_coding/LateralAcceleration.h>
#include <etsi_its_cam_msgs/LateralAcceleration.h>
#include <etsi_its_cam_conversion/convertLateralAccelerationValue.h>
#include <etsi_its_cam_conversion/convertAccelerationConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::LateralAcceleration convert_LateralAccelerationtoRos(const LateralAcceleration_t& _LateralAcceleration_in)
	{
		etsi_its_cam_msgs::LateralAcceleration LateralAcceleration_out;
		LateralAcceleration_out.lateralAccelerationValue = convert_LateralAccelerationValuetoRos(_LateralAcceleration_in.lateralAccelerationValue);
		LateralAcceleration_out.lateralAccelerationConfidence = convert_AccelerationConfidencetoRos(_LateralAcceleration_in.lateralAccelerationConfidence);
		return LateralAcceleration_out;
	}
	LateralAcceleration_t convert_LateralAccelerationtoC(const etsi_its_cam_msgs::LateralAcceleration& _LateralAcceleration_in)
	{
		LateralAcceleration_t LateralAcceleration_out;
		memset(&LateralAcceleration_out, 0, sizeof(LateralAcceleration_t));
		LateralAcceleration_out.lateralAccelerationValue = convert_LateralAccelerationValuetoC(_LateralAcceleration_in.lateralAccelerationValue);
		LateralAcceleration_out.lateralAccelerationConfidence = convert_AccelerationConfidencetoC(_LateralAcceleration_in.lateralAccelerationConfidence);
		return LateralAcceleration_out;
	}
}