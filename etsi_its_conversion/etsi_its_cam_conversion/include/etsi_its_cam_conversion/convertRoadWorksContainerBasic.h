#pragma once

#include <etsi_its_cam_coding/RoadWorksContainerBasic.h>
#include <etsi_its_cam_conversion/convertRoadworksSubCauseCode.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertClosedLanes.h>
#include <etsi_its_cam_msgs/RoadWorksContainerBasic.h>


namespace etsi_its_cam_conversion {

void toRos_RoadWorksContainerBasic(const RoadWorksContainerBasic_t& in, etsi_its_cam_msgs::RoadWorksContainerBasic& out) {

  if (in.roadworksSubCauseCode) {
    toRos_RoadworksSubCauseCode(*in.roadworksSubCauseCode, out.roadworks_sub_cause_code);
    out.roadworks_sub_cause_code_is_present = true;
  }

  toRos_LightBarSirenInUse(in.lightBarSirenInUse, out.light_bar_siren_in_use);
  if (in.closedLanes) {
    toRos_ClosedLanes(*in.closedLanes, out.closed_lanes);
    out.closed_lanes_is_present = true;
  }

}

void toStruct_RoadWorksContainerBasic(const etsi_its_cam_msgs::RoadWorksContainerBasic& in, RoadWorksContainerBasic_t& out) {
    
  memset(&out, 0, sizeof(RoadWorksContainerBasic_t));

  if (in.roadworks_sub_cause_code_is_present) {
    RoadworksSubCauseCode_t roadworks_sub_cause_code;
    toStruct_RoadworksSubCauseCode(in.roadworks_sub_cause_code, roadworks_sub_cause_code);
    out.roadworksSubCauseCode = new RoadworksSubCauseCode_t(roadworks_sub_cause_code);
  }

  toStruct_LightBarSirenInUse(in.light_bar_siren_in_use, out.lightBarSirenInUse);
  if (in.closed_lanes_is_present) {
    ClosedLanes_t closed_lanes;
    toStruct_ClosedLanes(in.closed_lanes, closed_lanes);
    out.closedLanes = new ClosedLanes_t(closed_lanes);
  }

}

}