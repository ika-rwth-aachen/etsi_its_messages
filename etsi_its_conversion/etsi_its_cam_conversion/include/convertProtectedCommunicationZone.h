#pragma once

#include <ProtectedCommunicationZone.h>
#include <etsi_its_cam_msgs/ProtectedCommunicationZone.h>
#include <convertProtectedZoneType.h>
#include <convertTimestampIts.h>
#include <convertLatitude.h>
#include <convertLongitude.h>
#include <convertProtectedZoneRadius.h>
#include <convertProtectedZoneID.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::ProtectedCommunicationZone convert_ProtectedCommunicationZonetoRos(const ProtectedCommunicationZone_t& _ProtectedCommunicationZone_in)
	{
		etsi_its_cam_msgs::ProtectedCommunicationZone ProtectedCommunicationZone_out;
		ProtectedCommunicationZone_out.protectedZoneType = convert_ProtectedZoneTypetoRos(_ProtectedCommunicationZone_in.protectedZoneType);
		if(_ProtectedCommunicationZone_in.expiryTime)
		{
			ProtectedCommunicationZone_out.expiryTime = convert_TimestampItstoRos(*_ProtectedCommunicationZone_in.expiryTime);
			ProtectedCommunicationZone_out.expiryTime_isPresent = true;
		}
		ProtectedCommunicationZone_out.protectedZoneLatitude = convert_LatitudetoRos(_ProtectedCommunicationZone_in.protectedZoneLatitude);
		ProtectedCommunicationZone_out.protectedZoneLongitude = convert_LongitudetoRos(_ProtectedCommunicationZone_in.protectedZoneLongitude);
		if(_ProtectedCommunicationZone_in.protectedZoneRadius)
		{
			ProtectedCommunicationZone_out.protectedZoneRadius = convert_ProtectedZoneRadiustoRos(*_ProtectedCommunicationZone_in.protectedZoneRadius);
			ProtectedCommunicationZone_out.protectedZoneRadius_isPresent = true;
		}
		if(_ProtectedCommunicationZone_in.protectedZoneID)
		{
			ProtectedCommunicationZone_out.protectedZoneID = convert_ProtectedZoneIDtoRos(*_ProtectedCommunicationZone_in.protectedZoneID);
			ProtectedCommunicationZone_out.protectedZoneID_isPresent = true;
		}
		return ProtectedCommunicationZone_out;
	}
}