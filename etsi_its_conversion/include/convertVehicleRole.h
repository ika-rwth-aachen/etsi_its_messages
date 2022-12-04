#pragma once

#include <VehicleRole.h>
#include <etsi_its_cam_msgs/VehicleRole.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::VehicleRole convert_VehicleRoletoRos(const VehicleRole_t& _VehicleRole_in)
	{
		etsi_its_cam_msgs::VehicleRole VehicleRole_out;
		VehicleRole_out.value = _VehicleRole_in;
		return VehicleRole_out;
	}
}