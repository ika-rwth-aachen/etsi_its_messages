#pragma once

#include <CenDsrcTollingZoneID.h>
#include <etsi_its_cam_msgs/CenDsrcTollingZoneID.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::CenDsrcTollingZoneID convert_CenDsrcTollingZoneIDtoRos(const CenDsrcTollingZoneID_t& _CenDsrcTollingZoneID_in)
	{
		etsi_its_cam_msgs::CenDsrcTollingZoneID CenDsrcTollingZoneID_out;
		return CenDsrcTollingZoneID_out;
	}
}