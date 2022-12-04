#pragma once

#include <ItsPduHeader.h>
#include <etsi_its_cam_msgs/ItsPduHeader.h>
#include <primitives/convertINTEGER.h>
#include <convertStationID.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::ItsPduHeader convert_ItsPduHeadertoRos(const ItsPduHeader_t& _ItsPduHeader_in)
	{
		etsi_its_cam_msgs::ItsPduHeader ItsPduHeader_out;
		convert_toRos(_ItsPduHeader_in.protocolVersion, ItsPduHeader_out.protocolVersion);
		convert_toRos(_ItsPduHeader_in.messageID, ItsPduHeader_out.messageID);
		ItsPduHeader_out.stationID = convert_StationIDtoRos(_ItsPduHeader_in.stationID);
		return ItsPduHeader_out;
	}
}