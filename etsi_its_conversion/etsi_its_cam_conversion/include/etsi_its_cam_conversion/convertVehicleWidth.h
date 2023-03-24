#pragma once

#include <etsi_its_cam_coding/VehicleWidth.h>
#include <etsi_its_cam_msgs/VehicleWidth.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::VehicleWidth convert_VehicleWidthtoRos(const VehicleWidth_t& _VehicleWidth_in)
	{
		etsi_its_cam_msgs::VehicleWidth VehicleWidth_out;
		convert_toRos(_VehicleWidth_in, VehicleWidth_out.value);
		return VehicleWidth_out;
	}
}