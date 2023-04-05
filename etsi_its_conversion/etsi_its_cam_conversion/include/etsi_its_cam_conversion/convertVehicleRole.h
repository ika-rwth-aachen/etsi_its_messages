#pragma once

#include <etsi_its_cam_coding/VehicleRole.h>
#include <etsi_its_cam_msgs/VehicleRole.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::VehicleRole convert_VehicleRoletoRos(const VehicleRole_t& _VehicleRole_in)
	{
		etsi_its_cam_msgs::VehicleRole VehicleRole_out;
		VehicleRole_out.value = _VehicleRole_in;
		return VehicleRole_out;
	}
	VehicleRole_t convert_VehicleRoletoC(const etsi_its_cam_msgs::VehicleRole& _VehicleRole_in)
	{
		VehicleRole_t VehicleRole_out;
		memset(&VehicleRole_out, 0, sizeof(VehicleRole_t));
		VehicleRole_out = _VehicleRole_in.value;
		return VehicleRole_out;
	}
}