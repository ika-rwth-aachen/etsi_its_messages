#pragma once

#include <RoadWorksContainerBasic.h>
#include <etsi_its_cam_msgs/RoadWorksContainerBasic.h>
#include <convertRoadworksSubCauseCode.h>
#include <convertLightBarSirenInUse.h>
#include <convertClosedLanes.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::RoadWorksContainerBasic convert_RoadWorksContainerBasictoRos(const RoadWorksContainerBasic_t& _RoadWorksContainerBasic_in)
	{
		etsi_its_cam_msgs::RoadWorksContainerBasic RoadWorksContainerBasic_out;
		if(_RoadWorksContainerBasic_in.roadworksSubCauseCode)
		{
			RoadWorksContainerBasic_out.roadworksSubCauseCode = convert_RoadworksSubCauseCodetoRos(*_RoadWorksContainerBasic_in.roadworksSubCauseCode);
		}
		RoadWorksContainerBasic_out.lightBarSirenInUse = convert_LightBarSirenInUsetoRos(_RoadWorksContainerBasic_in.lightBarSirenInUse);
		if(_RoadWorksContainerBasic_in.closedLanes)
		{
			RoadWorksContainerBasic_out.closedLanes = convert_ClosedLanestoRos(*_RoadWorksContainerBasic_in.closedLanes);
		}
		return RoadWorksContainerBasic_out;
	}
}