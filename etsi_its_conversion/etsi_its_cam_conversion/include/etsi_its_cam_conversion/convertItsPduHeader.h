#pragma once

#include <etsi_its_cam_coding/ItsPduHeader.h>
#include <etsi_its_cam_msgs/ItsPduHeader.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_conversion/convertStationID.h>

namespace etsi_its_cam_conversion
{
	void convert_ItsPduHeadertoRos(const ItsPduHeader_t& _ItsPduHeader_in, etsi_its_cam_msgs::ItsPduHeader& _ItsPduHeader_out)
	{
		convert_toRos(_ItsPduHeader_in.protocolVersion, _ItsPduHeader_out.protocolVersion);
		convert_toRos(_ItsPduHeader_in.messageID, _ItsPduHeader_out.messageID);
		convert_StationIDtoRos(_ItsPduHeader_in.stationID, _ItsPduHeader_out.stationID);
	}
	void convert_ItsPduHeadertoC(const etsi_its_cam_msgs::ItsPduHeader& _ItsPduHeader_in, ItsPduHeader_t& _ItsPduHeader_out)
	{
		memset(&_ItsPduHeader_out, 0, sizeof(ItsPduHeader_t));
		convert_toC(_ItsPduHeader_in.protocolVersion, _ItsPduHeader_out.protocolVersion);
		convert_toC(_ItsPduHeader_in.messageID, _ItsPduHeader_out.messageID);
		convert_StationIDtoC(_ItsPduHeader_in.stationID, _ItsPduHeader_out.stationID);
	}
}