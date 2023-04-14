#pragma once

#include <etsi_its_cam_coding/RoadWorksContainerBasic.h>
#include <etsi_its_cam_msgs/RoadWorksContainerBasic.h>
#include <etsi_its_cam_conversion/convertRoadworksSubCauseCode.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertClosedLanes.h>

namespace etsi_its_cam_conversion
{
	void convert_RoadWorksContainerBasictoRos(const RoadWorksContainerBasic_t& _RoadWorksContainerBasic_in, etsi_its_cam_msgs::RoadWorksContainerBasic& _RoadWorksContainerBasic_out)
	{
		if(_RoadWorksContainerBasic_in.roadworksSubCauseCode)
		{
			convert_RoadworksSubCauseCodetoRos(*_RoadWorksContainerBasic_in.roadworksSubCauseCode, _RoadWorksContainerBasic_out.roadworksSubCauseCode);
			_RoadWorksContainerBasic_out.roadworksSubCauseCode_isPresent = true;
		}
		convert_LightBarSirenInUsetoRos(_RoadWorksContainerBasic_in.lightBarSirenInUse, _RoadWorksContainerBasic_out.lightBarSirenInUse);
		if(_RoadWorksContainerBasic_in.closedLanes)
		{
			convert_ClosedLanestoRos(*_RoadWorksContainerBasic_in.closedLanes, _RoadWorksContainerBasic_out.closedLanes);
			_RoadWorksContainerBasic_out.closedLanes_isPresent = true;
		}
	}
	void convert_RoadWorksContainerBasictoC(const etsi_its_cam_msgs::RoadWorksContainerBasic& _RoadWorksContainerBasic_in, RoadWorksContainerBasic_t& _RoadWorksContainerBasic_out)
	{
		memset(&_RoadWorksContainerBasic_out, 0, sizeof(RoadWorksContainerBasic_t));
		if(_RoadWorksContainerBasic_in.roadworksSubCauseCode_isPresent)
		{
			RoadworksSubCauseCode_t roadworksSubCauseCode;
			convert_RoadworksSubCauseCodetoC(_RoadWorksContainerBasic_in.roadworksSubCauseCode, roadworksSubCauseCode);
			_RoadWorksContainerBasic_out.roadworksSubCauseCode = new RoadworksSubCauseCode_t(roadworksSubCauseCode);
		}
		convert_LightBarSirenInUsetoC(_RoadWorksContainerBasic_in.lightBarSirenInUse, _RoadWorksContainerBasic_out.lightBarSirenInUse);
		if(_RoadWorksContainerBasic_in.closedLanes_isPresent)
		{
			ClosedLanes_t closedLanes;
			convert_ClosedLanestoC(_RoadWorksContainerBasic_in.closedLanes, closedLanes);
			_RoadWorksContainerBasic_out.closedLanes = new ClosedLanes_t(closedLanes);
		}
	}
}