#pragma once

#include <etsi_its_cam_coding/LateralAccelerationValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/LateralAccelerationValue.h>


namespace etsi_its_cam_conversion {

void toRos_LateralAccelerationValue(const LateralAccelerationValue_t& in, etsi_its_cam_msgs::LateralAccelerationValue& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_LateralAccelerationValue(const etsi_its_cam_msgs::LateralAccelerationValue& in, LateralAccelerationValue_t& out) {
    
  memset(&out, 0, sizeof(LateralAccelerationValue_t));
  toStruct_INTEGER(in.value, out);
}

}