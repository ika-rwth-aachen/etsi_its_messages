#pragma once

#include <etsi_its_cam_coding/PtActivationData.h>
#include <etsi_its_cam_conversion/primitives/convertOCTET_STRING.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/pt_activation_data.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/PtActivationData.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_PtActivationData(const PtActivationData_t& in, cam_msgs::PtActivationData& out) {

  toRos_OCTET_STRING(in, out.value);
}

void toStruct_PtActivationData(const cam_msgs::PtActivationData& in, PtActivationData_t& out) {

  memset(&out, 0, sizeof(PtActivationData_t));
  toStruct_OCTET_STRING(in.value, out);
}

}