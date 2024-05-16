//// ENUMERATED RequestResponseIndication


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/RequestResponseIndication.h>

#ifdef ROS1
#include <etsi_its_denm_msgs/RequestResponseIndication.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/request_response_indication.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_RequestResponseIndication(const RequestResponseIndication_t& in, denm_msgs::RequestResponseIndication& out) {
  out.value = in;
}

void toStruct_RequestResponseIndication(const denm_msgs::RequestResponseIndication& in, RequestResponseIndication_t& out) {
  memset(&out, 0, sizeof(RequestResponseIndication_t));

  out = in.value;
}

}
