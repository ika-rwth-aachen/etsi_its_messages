//// INTEGER CurvatureValue


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/CurvatureValue.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/CurvatureValue.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/curvature_value.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_CurvatureValue(const CurvatureValue_t& in, denm_msgs::CurvatureValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_CurvatureValue(const denm_msgs::CurvatureValue& in, CurvatureValue_t& out) {
  memset(&out, 0, sizeof(CurvatureValue_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
