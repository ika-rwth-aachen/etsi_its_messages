//// INTEGER InformationQuality


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/InformationQuality.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/InformationQuality.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/information_quality.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_InformationQuality(const InformationQuality_t& in, cam_msgs::InformationQuality& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_InformationQuality(const cam_msgs::InformationQuality& in, InformationQuality_t& out) {
  memset(&out, 0, sizeof(InformationQuality_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
