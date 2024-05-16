//// INTEGER TransmissionInterval


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/TransmissionInterval.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/TransmissionInterval.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/transmission_interval.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_TransmissionInterval(const TransmissionInterval_t& in, cam_msgs::TransmissionInterval& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_TransmissionInterval(const cam_msgs::TransmissionInterval& in, TransmissionInterval_t& out) {
  memset(&out, 0, sizeof(TransmissionInterval_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
