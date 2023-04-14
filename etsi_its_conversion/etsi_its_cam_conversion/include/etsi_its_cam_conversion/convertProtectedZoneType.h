#pragma once

#include <etsi_its_cam_coding/ProtectedZoneType.h>
#include <etsi_its_cam_msgs/ProtectedZoneType.h>

namespace etsi_its_cam_conversion
{
	void convert_ProtectedZoneTypetoRos(const ProtectedZoneType_t& _ProtectedZoneType_in, etsi_its_cam_msgs::ProtectedZoneType& _ProtectedZoneType_out)
	{
		_ProtectedZoneType_out.value = _ProtectedZoneType_in;
	}
	void convert_ProtectedZoneTypetoC(const etsi_its_cam_msgs::ProtectedZoneType& _ProtectedZoneType_in, ProtectedZoneType_t& _ProtectedZoneType_out)
	{
		memset(&_ProtectedZoneType_out, 0, sizeof(ProtectedZoneType_t));
		_ProtectedZoneType_out = _ProtectedZoneType_in.value;
	}
}