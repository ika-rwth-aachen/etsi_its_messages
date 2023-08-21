#pragma once

#include <etsi_its_cam_coding/StationID.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/station_id.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/StationID.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_StationID(const StationID_t& in, cam_msgs::StationID& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_StationID(const cam_msgs::StationID& in, StationID_t& out) {
    
  memset(&out, 0, sizeof(StationID_t));
  toStruct_INTEGER(in.value, out);
}

}