#pragma once

#include <ProtectedZoneID.h>
#include <etsi_its_cam_msgs/ProtectedZoneID.h>
#include <primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::ProtectedZoneID convert_ProtectedZoneIDtoRos(const ProtectedZoneID_t& _ProtectedZoneID_in)
	{
		etsi_its_cam_msgs::ProtectedZoneID ProtectedZoneID_out;
		convert_toRos(_ProtectedZoneID_in, ProtectedZoneID_out.value);
		return ProtectedZoneID_out;
	}
}