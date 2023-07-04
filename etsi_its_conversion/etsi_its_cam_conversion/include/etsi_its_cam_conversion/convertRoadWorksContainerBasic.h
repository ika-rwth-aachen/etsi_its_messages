#pragma once

#include <etsi_its_cam_coding/RoadWorksContainerBasic.h>
#include <etsi_its_cam_conversion/convertRoadworksSubCauseCode.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertClosedLanes.h>
#include <etsi_its_cam_msgs/RoadWorksContainerBasic.h>


namespace etsi_its_cam_conversion {

void toRos_RoadWorksContainerBasic(const RoadWorksContainerBasic_t& in, etsi_its_cam_msgs::RoadWorksContainerBasic& out) {

  if (in.roadworksSubCauseCode) {
    toRos_RoadworksSubCauseCode(*in.roadworksSubCauseCode, out.roadworksSubCauseCode);
    out.roadworksSubCauseCode_isPresent = true;
  }

  toRos_LightBarSirenInUse(in.lightBarSirenInUse, out.lightBarSirenInUse);
  if (in.closedLanes) {
    toRos_ClosedLanes(*in.closedLanes, out.closedLanes);
    out.closedLanes_isPresent = true;
  }

}

void toStruct_RoadWorksContainerBasic(const etsi_its_cam_msgs::RoadWorksContainerBasic& in, RoadWorksContainerBasic_t& out) {
    
  memset(&out, 0, sizeof(RoadWorksContainerBasic_t));

  if (in.roadworksSubCauseCode_isPresent) {
    RoadworksSubCauseCode_t roadworksSubCauseCode;
    toStruct_RoadworksSubCauseCode(in.roadworksSubCauseCode, roadworksSubCauseCode);
    out.roadworksSubCauseCode = new RoadworksSubCauseCode_t(roadworksSubCauseCode);
  }

  toStruct_LightBarSirenInUse(in.lightBarSirenInUse, out.lightBarSirenInUse);
  if (in.closedLanes_isPresent) {
    ClosedLanes_t closedLanes;
    toStruct_ClosedLanes(in.closedLanes, closedLanes);
    out.closedLanes = new ClosedLanes_t(closedLanes);
  }

}

}