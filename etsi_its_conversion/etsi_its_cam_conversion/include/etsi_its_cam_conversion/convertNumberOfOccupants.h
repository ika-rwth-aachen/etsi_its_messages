//// INTEGER NumberOfOccupants


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/NumberOfOccupants.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/NumberOfOccupants.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/number_of_occupants.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_NumberOfOccupants(const NumberOfOccupants_t& in, cam_msgs::NumberOfOccupants& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_NumberOfOccupants(const cam_msgs::NumberOfOccupants& in, NumberOfOccupants_t& out) {
  memset(&out, 0, sizeof(NumberOfOccupants_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
