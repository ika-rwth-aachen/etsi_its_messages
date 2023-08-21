#pragma once

#include <etsi_its_cam_coding/BasicContainer.h>
#include <etsi_its_cam_conversion/convertStationType.h>
#include <etsi_its_cam_conversion/convertReferencePosition.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/basic_container.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/BasicContainer.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_BasicContainer(const BasicContainer_t& in, cam_msgs::BasicContainer& out) {

  toRos_StationType(in.stationType, out.station_type);
  toRos_ReferencePosition(in.referencePosition, out.reference_position);
}

void toStruct_BasicContainer(const cam_msgs::BasicContainer& in, BasicContainer_t& out) {
    
  memset(&out, 0, sizeof(BasicContainer_t));

  toStruct_StationType(in.station_type, out.stationType);
  toStruct_ReferencePosition(in.reference_position, out.referencePosition);
}

}