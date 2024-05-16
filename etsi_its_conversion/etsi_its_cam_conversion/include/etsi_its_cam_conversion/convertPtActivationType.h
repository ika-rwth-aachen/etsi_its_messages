//// INTEGER PtActivationType


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/PtActivationType.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/PtActivationType.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/pt_activation_type.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_PtActivationType(const PtActivationType_t& in, cam_msgs::PtActivationType& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PtActivationType(const cam_msgs::PtActivationType& in, PtActivationType_t& out) {
  memset(&out, 0, sizeof(PtActivationType_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
