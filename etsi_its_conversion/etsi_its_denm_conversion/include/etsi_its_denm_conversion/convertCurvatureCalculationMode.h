//// ENUMERATED CurvatureCalculationMode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/CurvatureCalculationMode.h>

#ifdef ROS1
#include <etsi_its_denm_msgs/CurvatureCalculationMode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/curvature_calculation_mode.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_CurvatureCalculationMode(const CurvatureCalculationMode_t& in, denm_msgs::CurvatureCalculationMode& out) {
  out.value = in;
}

void toStruct_CurvatureCalculationMode(const denm_msgs::CurvatureCalculationMode& in, CurvatureCalculationMode_t& out) {
  memset(&out, 0, sizeof(CurvatureCalculationMode_t));

  out = in.value;
}

}
