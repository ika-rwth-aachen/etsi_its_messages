#pragma once

#include <etsi_its_cam_coding/AccelerationControl.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/acceleration_control.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/AccelerationControl.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_AccelerationControl(const AccelerationControl_t& in, cam_msgs::AccelerationControl& out) {

  toRos_BIT_STRING(in, out.value);
  out.bits_unused = in.bits_unused;
}

void toStruct_AccelerationControl(const cam_msgs::AccelerationControl& in, AccelerationControl_t& out) {

  memset(&out, 0, sizeof(AccelerationControl_t));
  toStruct_BIT_STRING(in.value, out);
  out.bits_unused = in.bits_unused;
}

}