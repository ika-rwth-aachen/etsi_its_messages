#pragma once

#include <RoadworksSubCauseCode.h>
#include <etsi_its_cam_msgs/RoadworksSubCauseCode.h>
#include <primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::RoadworksSubCauseCode convert_RoadworksSubCauseCodetoRos(const RoadworksSubCauseCode_t& _RoadworksSubCauseCode_in)
	{
		etsi_its_cam_msgs::RoadworksSubCauseCode RoadworksSubCauseCode_out;
		convert_toRos(_RoadworksSubCauseCode_in, RoadworksSubCauseCode_out.value);
		return RoadworksSubCauseCode_out;
	}
}