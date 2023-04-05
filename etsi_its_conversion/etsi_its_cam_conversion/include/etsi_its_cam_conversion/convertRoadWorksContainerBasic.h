#pragma once

#include <etsi_its_cam_coding/RoadWorksContainerBasic.h>
#include <etsi_its_cam_msgs/RoadWorksContainerBasic.h>
#include <etsi_its_cam_conversion/convertRoadworksSubCauseCode.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertClosedLanes.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::RoadWorksContainerBasic convert_RoadWorksContainerBasictoRos(const RoadWorksContainerBasic_t& _RoadWorksContainerBasic_in)
	{
		etsi_its_cam_msgs::RoadWorksContainerBasic RoadWorksContainerBasic_out;
		if(_RoadWorksContainerBasic_in.roadworksSubCauseCode)
		{
			RoadWorksContainerBasic_out.roadworksSubCauseCode = convert_RoadworksSubCauseCodetoRos(*_RoadWorksContainerBasic_in.roadworksSubCauseCode);
			RoadWorksContainerBasic_out.roadworksSubCauseCode_isPresent = true;
		}
		RoadWorksContainerBasic_out.lightBarSirenInUse = convert_LightBarSirenInUsetoRos(_RoadWorksContainerBasic_in.lightBarSirenInUse);
		if(_RoadWorksContainerBasic_in.closedLanes)
		{
			RoadWorksContainerBasic_out.closedLanes = convert_ClosedLanestoRos(*_RoadWorksContainerBasic_in.closedLanes);
			RoadWorksContainerBasic_out.closedLanes_isPresent = true;
		}
		return RoadWorksContainerBasic_out;
	}
	RoadWorksContainerBasic_t convert_RoadWorksContainerBasictoC(const etsi_its_cam_msgs::RoadWorksContainerBasic& _RoadWorksContainerBasic_in)
	{
		RoadWorksContainerBasic_t RoadWorksContainerBasic_out;
		memset(&RoadWorksContainerBasic_out, 0, sizeof(RoadWorksContainerBasic_t));
		if(_RoadWorksContainerBasic_in.roadworksSubCauseCode_isPresent)
		{
			auto roadworksSubCauseCode = convert_RoadworksSubCauseCodetoC(_RoadWorksContainerBasic_in.roadworksSubCauseCode);
			RoadWorksContainerBasic_out.roadworksSubCauseCode = new RoadworksSubCauseCode_t(roadworksSubCauseCode);
		}
		RoadWorksContainerBasic_out.lightBarSirenInUse = convert_LightBarSirenInUsetoC(_RoadWorksContainerBasic_in.lightBarSirenInUse);
		if(_RoadWorksContainerBasic_in.closedLanes_isPresent)
		{
			auto closedLanes = convert_ClosedLanestoC(_RoadWorksContainerBasic_in.closedLanes);
			RoadWorksContainerBasic_out.closedLanes = new ClosedLanes_t(closedLanes);
		}
		return RoadWorksContainerBasic_out;
	}
}