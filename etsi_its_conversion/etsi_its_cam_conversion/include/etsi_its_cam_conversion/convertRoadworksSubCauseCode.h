//// INTEGER RoadworksSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/RoadworksSubCauseCode.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/RoadworksSubCauseCode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/roadworks_sub_cause_code.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_RoadworksSubCauseCode(const RoadworksSubCauseCode_t& in, cam_msgs::RoadworksSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_RoadworksSubCauseCode(const cam_msgs::RoadworksSubCauseCode& in, RoadworksSubCauseCode_t& out) {
  memset(&out, 0, sizeof(RoadworksSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
