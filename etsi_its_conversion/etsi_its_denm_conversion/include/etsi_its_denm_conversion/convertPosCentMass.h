//// INTEGER PosCentMass


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/PosCentMass.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PosCentMass.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/pos_cent_mass.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PosCentMass(const PosCentMass_t& in, denm_msgs::PosCentMass& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_PosCentMass(const denm_msgs::PosCentMass& in, PosCentMass_t& out) {
  memset(&out, 0, sizeof(PosCentMass_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
