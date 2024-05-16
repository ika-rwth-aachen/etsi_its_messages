//// ENUMERATED DangerousGoodsBasic


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/DangerousGoodsBasic.h>

#ifdef ROS1
#include <etsi_its_denm_msgs/DangerousGoodsBasic.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/dangerous_goods_basic.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_DangerousGoodsBasic(const DangerousGoodsBasic_t& in, denm_msgs::DangerousGoodsBasic& out) {
  out.value = in;
}

void toStruct_DangerousGoodsBasic(const denm_msgs::DangerousGoodsBasic& in, DangerousGoodsBasic_t& out) {
  memset(&out, 0, sizeof(DangerousGoodsBasic_t));

  out = in.value;
}

}
