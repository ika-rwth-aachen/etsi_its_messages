//// INTEGER ValidityDuration


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/ValidityDuration.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/ValidityDuration.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/validity_duration.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_ValidityDuration(const ValidityDuration_t& in, cam_msgs::ValidityDuration& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_ValidityDuration(const cam_msgs::ValidityDuration& in, ValidityDuration_t& out) {
  memset(&out, 0, sizeof(ValidityDuration_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
