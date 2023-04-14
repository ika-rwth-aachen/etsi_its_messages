#pragma once

#include <etsi_its_cam_coding/CenDsrcTollingZone.h>
#include <etsi_its_cam_msgs/CenDsrcTollingZone.h>
#include <etsi_its_cam_conversion/convertLatitude.h>
#include <etsi_its_cam_conversion/convertLongitude.h>
#include <etsi_its_cam_conversion/convertCenDsrcTollingZoneID.h>

namespace etsi_its_cam_conversion
{
	void convert_CenDsrcTollingZonetoRos(const CenDsrcTollingZone_t& _CenDsrcTollingZone_in, etsi_its_cam_msgs::CenDsrcTollingZone& _CenDsrcTollingZone_out)
	{
		convert_LatitudetoRos(_CenDsrcTollingZone_in.protectedZoneLatitude, _CenDsrcTollingZone_out.protectedZoneLatitude);
		convert_LongitudetoRos(_CenDsrcTollingZone_in.protectedZoneLongitude, _CenDsrcTollingZone_out.protectedZoneLongitude);
		if(_CenDsrcTollingZone_in.cenDsrcTollingZoneID)
		{
			convert_CenDsrcTollingZoneIDtoRos(*_CenDsrcTollingZone_in.cenDsrcTollingZoneID, _CenDsrcTollingZone_out.cenDsrcTollingZoneID);
			_CenDsrcTollingZone_out.cenDsrcTollingZoneID_isPresent = true;
		}
	}
	void convert_CenDsrcTollingZonetoC(const etsi_its_cam_msgs::CenDsrcTollingZone& _CenDsrcTollingZone_in, CenDsrcTollingZone_t& _CenDsrcTollingZone_out)
	{
		memset(&_CenDsrcTollingZone_out, 0, sizeof(CenDsrcTollingZone_t));
		convert_LatitudetoC(_CenDsrcTollingZone_in.protectedZoneLatitude, _CenDsrcTollingZone_out.protectedZoneLatitude);
		convert_LongitudetoC(_CenDsrcTollingZone_in.protectedZoneLongitude, _CenDsrcTollingZone_out.protectedZoneLongitude);
		if(_CenDsrcTollingZone_in.cenDsrcTollingZoneID_isPresent)
		{
			CenDsrcTollingZoneID_t cenDsrcTollingZoneID;
			convert_CenDsrcTollingZoneIDtoC(_CenDsrcTollingZone_in.cenDsrcTollingZoneID, cenDsrcTollingZoneID);
			_CenDsrcTollingZone_out.cenDsrcTollingZoneID = new CenDsrcTollingZoneID_t(cenDsrcTollingZoneID);
		}
	}
}