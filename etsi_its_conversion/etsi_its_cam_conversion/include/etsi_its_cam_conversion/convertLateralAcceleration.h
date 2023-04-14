#pragma once

#include <etsi_its_cam_coding/LateralAcceleration.h>
#include <etsi_its_cam_msgs/LateralAcceleration.h>
#include <etsi_its_cam_conversion/convertLateralAccelerationValue.h>
#include <etsi_its_cam_conversion/convertAccelerationConfidence.h>

namespace etsi_its_cam_conversion
{
	void convert_LateralAccelerationtoRos(const LateralAcceleration_t& _LateralAcceleration_in, etsi_its_cam_msgs::LateralAcceleration& _LateralAcceleration_out)
	{
		convert_LateralAccelerationValuetoRos(_LateralAcceleration_in.lateralAccelerationValue, _LateralAcceleration_out.lateralAccelerationValue);
		convert_AccelerationConfidencetoRos(_LateralAcceleration_in.lateralAccelerationConfidence, _LateralAcceleration_out.lateralAccelerationConfidence);
	}
	void convert_LateralAccelerationtoC(const etsi_its_cam_msgs::LateralAcceleration& _LateralAcceleration_in, LateralAcceleration_t& _LateralAcceleration_out)
	{
		memset(&_LateralAcceleration_out, 0, sizeof(LateralAcceleration_t));
		convert_LateralAccelerationValuetoC(_LateralAcceleration_in.lateralAccelerationValue, _LateralAcceleration_out.lateralAccelerationValue);
		convert_AccelerationConfidencetoC(_LateralAcceleration_in.lateralAccelerationConfidence, _LateralAcceleration_out.lateralAccelerationConfidence);
	}
}