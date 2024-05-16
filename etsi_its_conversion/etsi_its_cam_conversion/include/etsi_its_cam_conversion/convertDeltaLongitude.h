//// INTEGER DeltaLongitude


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/DeltaLongitude.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/DeltaLongitude.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/delta_longitude.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_DeltaLongitude(const DeltaLongitude_t& in, cam_msgs::DeltaLongitude& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_DeltaLongitude(const cam_msgs::DeltaLongitude& in, DeltaLongitude_t& out) {
  memset(&out, 0, sizeof(DeltaLongitude_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
