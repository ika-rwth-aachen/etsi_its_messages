#pragma once

#include <etsi_its_cam_coding/CurvatureValue.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/CurvatureValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/curvature_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_CurvatureValue(const CurvatureValue_t& in, cam_msgs::CurvatureValue& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_CurvatureValue(const cam_msgs::CurvatureValue& in, CurvatureValue_t& out) {

  memset(&out, 0, sizeof(CurvatureValue_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}