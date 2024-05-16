//// INTEGER PosCentMass


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/PosCentMass.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/PosCentMass.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/pos_cent_mass.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_PosCentMass(const PosCentMass_t& in, cam_msgs::PosCentMass& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PosCentMass(const cam_msgs::PosCentMass& in, PosCentMass_t& out) {
  memset(&out, 0, sizeof(PosCentMass_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
