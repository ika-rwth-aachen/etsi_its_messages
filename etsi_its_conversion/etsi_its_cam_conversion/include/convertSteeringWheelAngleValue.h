#pragma once

#include <SteeringWheelAngleValue.h>
#include <etsi_its_cam_msgs/SteeringWheelAngleValue.h>
#include <primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SteeringWheelAngleValue convert_SteeringWheelAngleValuetoRos(const SteeringWheelAngleValue_t& _SteeringWheelAngleValue_in)
	{
		etsi_its_cam_msgs::SteeringWheelAngleValue SteeringWheelAngleValue_out;
		convert_toRos(_SteeringWheelAngleValue_in, SteeringWheelAngleValue_out.value);
		return SteeringWheelAngleValue_out;
	}
}