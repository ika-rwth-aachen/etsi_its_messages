#pragma once

#include <etsi_its_cam_coding/CenDsrcTollingZoneID.h>
#include <etsi_its_cam_msgs/CenDsrcTollingZoneID.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::CenDsrcTollingZoneID convert_CenDsrcTollingZoneIDtoRos(const CenDsrcTollingZoneID_t& _CenDsrcTollingZoneID_in)
	{
		etsi_its_cam_msgs::CenDsrcTollingZoneID CenDsrcTollingZoneID_out;
		return CenDsrcTollingZoneID_out;
	}
	CenDsrcTollingZoneID_t convert_CenDsrcTollingZoneIDtoC(const etsi_its_cam_msgs::CenDsrcTollingZoneID& _CenDsrcTollingZoneID_in)
	{
		CenDsrcTollingZoneID_t CenDsrcTollingZoneID_out;
		memset(&CenDsrcTollingZoneID_out, 0, sizeof(CenDsrcTollingZoneID_t));
		return CenDsrcTollingZoneID_out;
	}
}