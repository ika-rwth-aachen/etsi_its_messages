#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngleValue.h>
#include <etsi_its_cam_msgs/SteeringWheelAngleValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SteeringWheelAngleValue convert_SteeringWheelAngleValuetoRos(const SteeringWheelAngleValue_t& _SteeringWheelAngleValue_in)
	{
		etsi_its_cam_msgs::SteeringWheelAngleValue SteeringWheelAngleValue_out;
		convert_toRos(_SteeringWheelAngleValue_in, SteeringWheelAngleValue_out.value);
		return SteeringWheelAngleValue_out;
	}
	SteeringWheelAngleValue_t convert_SteeringWheelAngleValuetoC(const etsi_its_cam_msgs::SteeringWheelAngleValue& _SteeringWheelAngleValue_in)
	{
		SteeringWheelAngleValue_t SteeringWheelAngleValue_out;
		memset(&SteeringWheelAngleValue_out, 0, sizeof(SteeringWheelAngleValue_t));
		convert_toC(_SteeringWheelAngleValue_in.value, SteeringWheelAngleValue_out);
		return SteeringWheelAngleValue_out;
	}
}