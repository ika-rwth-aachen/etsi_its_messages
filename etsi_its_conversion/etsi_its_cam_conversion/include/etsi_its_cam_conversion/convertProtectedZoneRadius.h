#pragma once

#include <etsi_its_cam_coding/ProtectedZoneRadius.h>
#include <etsi_its_cam_msgs/ProtectedZoneRadius.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::ProtectedZoneRadius convert_ProtectedZoneRadiustoRos(const ProtectedZoneRadius_t& _ProtectedZoneRadius_in)
	{
		etsi_its_cam_msgs::ProtectedZoneRadius ProtectedZoneRadius_out;
		convert_toRos(_ProtectedZoneRadius_in, ProtectedZoneRadius_out.value);
		return ProtectedZoneRadius_out;
	}
}