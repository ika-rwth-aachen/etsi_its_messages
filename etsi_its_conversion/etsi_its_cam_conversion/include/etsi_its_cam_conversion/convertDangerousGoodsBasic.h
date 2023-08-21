#pragma once

#include <etsi_its_cam_coding/DangerousGoodsBasic.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/dangerous_goods_basic.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/DangerousGoodsBasic.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_DangerousGoodsBasic(const DangerousGoodsBasic_t& in, cam_msgs::DangerousGoodsBasic& out) {

  out.value = in;
}

void toStruct_DangerousGoodsBasic(const cam_msgs::DangerousGoodsBasic& in, DangerousGoodsBasic_t& out) {
    
  memset(&out, 0, sizeof(DangerousGoodsBasic_t));
  out = in.value;
}

}