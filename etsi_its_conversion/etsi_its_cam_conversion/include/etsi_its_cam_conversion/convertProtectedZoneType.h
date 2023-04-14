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
	ProtectedZoneType_t convert_ProtectedZoneTypetoC(const etsi_its_cam_msgs::ProtectedZoneType& _ProtectedZoneType_in)
	{
		ProtectedZoneType_t ProtectedZoneType_out;
		memset(&ProtectedZoneType_out, 0, sizeof(ProtectedZoneType_t));
		ProtectedZoneType_out = _ProtectedZoneType_in.value;
		return ProtectedZoneType_out;
	}
}