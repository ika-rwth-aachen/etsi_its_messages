#pragma once

#include <etsi_its_cam_coding/ProtectedCommunicationZone.h>
#include <etsi_its_cam_msgs/ProtectedCommunicationZone.h>
#include <etsi_its_cam_conversion/convertProtectedZoneType.h>
#include <etsi_its_cam_conversion/convertTimestampIts.h>
#include <etsi_its_cam_conversion/convertLatitude.h>
#include <etsi_its_cam_conversion/convertLongitude.h>
#include <etsi_its_cam_conversion/convertProtectedZoneRadius.h>
#include <etsi_its_cam_conversion/convertProtectedZoneID.h>

namespace etsi_its_cam_conversion
{
	void convert_ProtectedCommunicationZonetoRos(const ProtectedCommunicationZone_t& _ProtectedCommunicationZone_in, etsi_its_cam_msgs::ProtectedCommunicationZone& _ProtectedCommunicationZone_out)
	{
		convert_ProtectedZoneTypetoRos(_ProtectedCommunicationZone_in.protectedZoneType, _ProtectedCommunicationZone_out.protectedZoneType);
		if(_ProtectedCommunicationZone_in.expiryTime)
		{
			convert_TimestampItstoRos(*_ProtectedCommunicationZone_in.expiryTime, _ProtectedCommunicationZone_out.expiryTime);
			_ProtectedCommunicationZone_out.expiryTime_isPresent = true;
		}
		convert_LatitudetoRos(_ProtectedCommunicationZone_in.protectedZoneLatitude, _ProtectedCommunicationZone_out.protectedZoneLatitude);
		convert_LongitudetoRos(_ProtectedCommunicationZone_in.protectedZoneLongitude, _ProtectedCommunicationZone_out.protectedZoneLongitude);
		if(_ProtectedCommunicationZone_in.protectedZoneRadius)
		{
			convert_ProtectedZoneRadiustoRos(*_ProtectedCommunicationZone_in.protectedZoneRadius, _ProtectedCommunicationZone_out.protectedZoneRadius);
			_ProtectedCommunicationZone_out.protectedZoneRadius_isPresent = true;
		}
		if(_ProtectedCommunicationZone_in.protectedZoneID)
		{
			convert_ProtectedZoneIDtoRos(*_ProtectedCommunicationZone_in.protectedZoneID, _ProtectedCommunicationZone_out.protectedZoneID);
			_ProtectedCommunicationZone_out.protectedZoneID_isPresent = true;
		}
	}
	void convert_ProtectedCommunicationZonetoC(const etsi_its_cam_msgs::ProtectedCommunicationZone& _ProtectedCommunicationZone_in, ProtectedCommunicationZone_t& _ProtectedCommunicationZone_out)
	{
		memset(&_ProtectedCommunicationZone_out, 0, sizeof(ProtectedCommunicationZone_t));
		convert_ProtectedZoneTypetoC(_ProtectedCommunicationZone_in.protectedZoneType, _ProtectedCommunicationZone_out.protectedZoneType);
		if(_ProtectedCommunicationZone_in.expiryTime_isPresent)
		{
			TimestampIts_t expiryTime;
			convert_TimestampItstoC(_ProtectedCommunicationZone_in.expiryTime, expiryTime);
			_ProtectedCommunicationZone_out.expiryTime = new TimestampIts_t(expiryTime);
		}
		convert_LatitudetoC(_ProtectedCommunicationZone_in.protectedZoneLatitude, _ProtectedCommunicationZone_out.protectedZoneLatitude);
		convert_LongitudetoC(_ProtectedCommunicationZone_in.protectedZoneLongitude, _ProtectedCommunicationZone_out.protectedZoneLongitude);
		if(_ProtectedCommunicationZone_in.protectedZoneRadius_isPresent)
		{
			ProtectedZoneRadius_t protectedZoneRadius;
			convert_ProtectedZoneRadiustoC(_ProtectedCommunicationZone_in.protectedZoneRadius, protectedZoneRadius);
			_ProtectedCommunicationZone_out.protectedZoneRadius = new ProtectedZoneRadius_t(protectedZoneRadius);
		}
		if(_ProtectedCommunicationZone_in.protectedZoneID_isPresent)
		{
			ProtectedZoneID_t protectedZoneID;
			convert_ProtectedZoneIDtoC(_ProtectedCommunicationZone_in.protectedZoneID, protectedZoneID);
			_ProtectedCommunicationZone_out.protectedZoneID = new ProtectedZoneID_t(protectedZoneID);
		}
	}
}