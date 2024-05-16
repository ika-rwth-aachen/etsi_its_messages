//// BOOLEAN EmbarkationStatus


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/EmbarkationStatus.h>
#include <etsi_its_denm_coding/BOOLEAN.h>
#include <etsi_its_primitives_conversion/convertBOOLEAN.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/EmbarkationStatus.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/embarkation_status.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_EmbarkationStatus(const EmbarkationStatus_t& in, denm_msgs::EmbarkationStatus& out) {
  etsi_its_primitives_conversion::toRos_BOOLEAN(in, out.value);
}

void toStruct_EmbarkationStatus(const denm_msgs::EmbarkationStatus& in, EmbarkationStatus_t& out) {
  memset(&out, 0, sizeof(EmbarkationStatus_t));

  etsi_its_primitives_conversion::toStruct_BOOLEAN(in.value, out);
}

}
