#pragma once

#include <etsi_its_cam_coding/CurvatureCalculationMode.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/CurvatureCalculationMode.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/curvature_calculation_mode.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_CurvatureCalculationMode(const CurvatureCalculationMode_t& in, cam_msgs::CurvatureCalculationMode& out) {

  out.value = in;
}

void toStruct_CurvatureCalculationMode(const cam_msgs::CurvatureCalculationMode& in, CurvatureCalculationMode_t& out) {

  memset(&out, 0, sizeof(CurvatureCalculationMode_t));
  out = in.value;
}

}