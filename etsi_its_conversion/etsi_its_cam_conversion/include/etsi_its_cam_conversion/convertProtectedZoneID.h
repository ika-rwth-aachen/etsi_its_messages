#pragma once

#include <etsi_its_cam_coding/ProtectedZoneID.h>
#include <etsi_its_cam_msgs/ProtectedZoneID.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_ProtectedZoneIDtoRos(const ProtectedZoneID_t& _ProtectedZoneID_in, etsi_its_cam_msgs::ProtectedZoneID& _ProtectedZoneID_out)
	{
		convert_toRos(_ProtectedZoneID_in, _ProtectedZoneID_out.value);
	}
	void convert_ProtectedZoneIDtoC(const etsi_its_cam_msgs::ProtectedZoneID& _ProtectedZoneID_in, ProtectedZoneID_t& _ProtectedZoneID_out)
	{
		memset(&_ProtectedZoneID_out, 0, sizeof(ProtectedZoneID_t));
		convert_toC(_ProtectedZoneID_in.value, _ProtectedZoneID_out);
	}
}