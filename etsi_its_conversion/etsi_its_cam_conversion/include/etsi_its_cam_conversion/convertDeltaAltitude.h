#pragma once

#include <etsi_its_cam_coding/DeltaAltitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/delta_altitude.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/DeltaAltitude.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_DeltaAltitude(const DeltaAltitude_t& in, cam_msgs::DeltaAltitude& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_DeltaAltitude(const cam_msgs::DeltaAltitude& in, DeltaAltitude_t& out) {
    
  memset(&out, 0, sizeof(DeltaAltitude_t));
  toStruct_INTEGER(in.value, out);
}

}