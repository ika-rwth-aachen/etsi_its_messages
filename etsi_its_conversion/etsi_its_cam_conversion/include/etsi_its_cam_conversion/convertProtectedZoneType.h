#pragma once

#include <etsi_its_cam_coding/ProtectedZoneType.h>
#include <etsi_its_cam_msgs/ProtectedZoneType.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::ProtectedZoneType convert_ProtectedZoneTypetoRos(const ProtectedZoneType_t& _ProtectedZoneType_in)
	{
		etsi_its_cam_msgs::ProtectedZoneType ProtectedZoneType_out;
		ProtectedZoneType_out.value = _ProtectedZoneType_in;
		return ProtectedZoneType_out;
	}
}