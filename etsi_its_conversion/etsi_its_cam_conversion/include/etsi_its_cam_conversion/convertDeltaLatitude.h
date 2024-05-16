//// INTEGER DeltaLatitude


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/DeltaLatitude.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/DeltaLatitude.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/delta_latitude.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_DeltaLatitude(const DeltaLatitude_t& in, cam_msgs::DeltaLatitude& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_DeltaLatitude(const cam_msgs::DeltaLatitude& in, DeltaLatitude_t& out) {
  memset(&out, 0, sizeof(DeltaLatitude_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
