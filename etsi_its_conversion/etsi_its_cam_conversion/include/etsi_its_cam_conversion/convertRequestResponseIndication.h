//// ENUMERATED RequestResponseIndication


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/RequestResponseIndication.h>

#ifdef ROS1
#include <etsi_its_cam_msgs/RequestResponseIndication.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/request_response_indication.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_RequestResponseIndication(const RequestResponseIndication_t& in, cam_msgs::RequestResponseIndication& out) {
  out.value = in;
}

void toStruct_RequestResponseIndication(const cam_msgs::RequestResponseIndication& in, RequestResponseIndication_t& out) {
  memset(&out, 0, sizeof(RequestResponseIndication_t));

  out = in.value;
}

}
