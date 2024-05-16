//// INTEGER PosLonCarr


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/PosLonCarr.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/PosLonCarr.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/pos_lon_carr.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_PosLonCarr(const PosLonCarr_t& in, cam_msgs::PosLonCarr& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PosLonCarr(const cam_msgs::PosLonCarr& in, PosLonCarr_t& out) {
  memset(&out, 0, sizeof(PosLonCarr_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
