#pragma once

#include <etsi_its_cam_coding/VehicleRole.h>
#include <etsi_its_cam_msgs/VehicleRole.h>

namespace etsi_its_cam_conversion
{
	void convert_VehicleRoletoRos(const VehicleRole_t& _VehicleRole_in, etsi_its_cam_msgs::VehicleRole& _VehicleRole_out)
	{
		_VehicleRole_out.value = _VehicleRole_in;
	}
	void convert_VehicleRoletoC(const etsi_its_cam_msgs::VehicleRole& _VehicleRole_in, VehicleRole_t& _VehicleRole_out)
	{
		memset(&_VehicleRole_out, 0, sizeof(VehicleRole_t));
		_VehicleRole_out = _VehicleRole_in.value;
	}
}