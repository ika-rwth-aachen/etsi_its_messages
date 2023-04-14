#pragma once

#include <etsi_its_cam_coding/BasicContainer.h>
#include <etsi_its_cam_msgs/BasicContainer.h>
#include <etsi_its_cam_conversion/convertStationType.h>
#include <etsi_its_cam_conversion/convertReferencePosition.h>

namespace etsi_its_cam_conversion {
  
void toRos_BasicContainer(const BasicContainer_t& in, etsi_its_cam_msgs::BasicContainer& out) {
  toRos_StationType(in.stationType, out.stationType);
  toRos_ReferencePosition(in.referencePosition, out.referencePosition);
}

void toStruct_BasicContainer(const etsi_its_cam_msgs::BasicContainer& in, BasicContainer_t& out) {
  memset(&out, 0, sizeof(BasicContainer_t));
  toStruct_StationType(in.stationType, out.stationType);
  toStruct_ReferencePosition(in.referencePosition, out.referencePosition);
}

}