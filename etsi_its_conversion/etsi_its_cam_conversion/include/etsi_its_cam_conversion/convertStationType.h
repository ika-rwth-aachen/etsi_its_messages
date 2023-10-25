#pragma once

#include <etsi_its_cam_coding/StationType.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/StationType.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/station_type.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_StationType(const StationType_t& in, cam_msgs::StationType& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_StationType(const cam_msgs::StationType& in, StationType_t& out) {

  memset(&out, 0, sizeof(StationType_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}