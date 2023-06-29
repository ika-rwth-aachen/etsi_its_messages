#pragma once

#include <etsi_its_cam_coding/StationID.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/StationID.h>


namespace etsi_its_cam_conversion {

void toRos_StationID(const StationID_t& in, etsi_its_cam_msgs::StationID& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_StationID(const etsi_its_cam_msgs::StationID& in, StationID_t& out) {
    
  memset(&out, 0, sizeof(StationID_t));
  toStruct_INTEGER(in.value, out);
}

}