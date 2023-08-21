#pragma once

#include <etsi_its_cam_coding/BasicContainer.h>
#include <etsi_its_cam_conversion/convertStationType.h>
#include <etsi_its_cam_conversion/convertReferencePosition.h>
#include <etsi_its_cam_msgs/BasicContainer.h>


namespace etsi_its_cam_conversion {

void toRos_BasicContainer(const BasicContainer_t& in, etsi_its_cam_msgs::BasicContainer& out) {

  toRos_StationType(in.station_type, out.station_type);
  toRos_ReferencePosition(in.reference_position, out.reference_position);
}

void toStruct_BasicContainer(const etsi_its_cam_msgs::BasicContainer& in, BasicContainer_t& out) {
    
  memset(&out, 0, sizeof(BasicContainer_t));

  toStruct_StationType(in.station_type, out.station_type);
  toStruct_ReferencePosition(in.reference_position, out.reference_position);
}

}