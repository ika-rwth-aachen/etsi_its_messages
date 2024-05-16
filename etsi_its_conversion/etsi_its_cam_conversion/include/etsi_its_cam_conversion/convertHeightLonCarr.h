//// INTEGER HeightLonCarr


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/HeightLonCarr.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/HeightLonCarr.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/height_lon_carr.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_HeightLonCarr(const HeightLonCarr_t& in, cam_msgs::HeightLonCarr& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HeightLonCarr(const cam_msgs::HeightLonCarr& in, HeightLonCarr_t& out) {
  memset(&out, 0, sizeof(HeightLonCarr_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
