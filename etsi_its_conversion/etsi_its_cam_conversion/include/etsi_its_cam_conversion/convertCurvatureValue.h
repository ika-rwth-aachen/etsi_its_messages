#pragma once

#include <etsi_its_cam_coding/CurvatureValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/CurvatureValue.h>


namespace etsi_its_cam_conversion {

void toRos_CurvatureValue(const CurvatureValue_t& in, etsi_its_cam_msgs::CurvatureValue& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_CurvatureValue(const etsi_its_cam_msgs::CurvatureValue& in, CurvatureValue_t& out) {
    
  memset(&out, 0, sizeof(CurvatureValue_t));
  toStruct_INTEGER(in.value, out);
}

}