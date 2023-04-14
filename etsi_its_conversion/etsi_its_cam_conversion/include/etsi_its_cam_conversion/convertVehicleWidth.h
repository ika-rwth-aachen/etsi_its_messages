#pragma once

#include <etsi_its_cam_coding/VehicleWidth.h>
#include <etsi_its_cam_msgs/VehicleWidth.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_VehicleWidthtoRos(const VehicleWidth_t& _VehicleWidth_in, etsi_its_cam_msgs::VehicleWidth& _VehicleWidth_out)
	{
		convert_toRos(_VehicleWidth_in, _VehicleWidth_out.value);
	}
	void convert_VehicleWidthtoC(const etsi_its_cam_msgs::VehicleWidth& _VehicleWidth_in, VehicleWidth_t& _VehicleWidth_out)
	{
		memset(&_VehicleWidth_out, 0, sizeof(VehicleWidth_t));
		convert_toC(_VehicleWidth_in.value, _VehicleWidth_out);
	}
}