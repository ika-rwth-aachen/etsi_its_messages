//// SEQUENCE DENM


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/DENM.h>
#include <etsi_its_denm_conversion/convertItsPduHeader.h>
#include <etsi_its_denm_conversion/convertDecentralizedEnvironmentalNotificationMessage.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/DENM.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/denm.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_DENM(const DENM_t& in, denm_msgs::DENM& out) {
  toRos_ItsPduHeader(in.header, out.header);
  toRos_DecentralizedEnvironmentalNotificationMessage(in.denm, out.denm);
}

void toStruct_DENM(const denm_msgs::DENM& in, DENM_t& out) {
  memset(&out, 0, sizeof(DENM_t));

  toStruct_ItsPduHeader(in.header, out.header);
  toStruct_DecentralizedEnvironmentalNotificationMessage(in.denm, out.denm);
}

}
