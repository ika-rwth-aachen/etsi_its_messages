#pragma once

#include <etsi_its_cam_coding/ProtectedZoneID.h>
#include <etsi_its_cam_msgs/ProtectedZoneID.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::ProtectedZoneID convert_ProtectedZoneIDtoRos(const ProtectedZoneID_t& _ProtectedZoneID_in)
	{
		etsi_its_cam_msgs::ProtectedZoneID ProtectedZoneID_out;
		convert_toRos(_ProtectedZoneID_in, ProtectedZoneID_out.value);
		return ProtectedZoneID_out;
	}
	ProtectedZoneID_t convert_ProtectedZoneIDtoC(const etsi_its_cam_msgs::ProtectedZoneID& _ProtectedZoneID_in)
	{
		ProtectedZoneID_t ProtectedZoneID_out;
		memset(&ProtectedZoneID_out, 0, sizeof(ProtectedZoneID_t));
		convert_toC(_ProtectedZoneID_in.value, ProtectedZoneID_out);
		return ProtectedZoneID_out;
	}
}