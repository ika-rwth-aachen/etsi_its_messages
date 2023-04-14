#pragma once

#include <etsi_its_cam_coding/DangerousGoodsBasic.h>
#include <etsi_its_cam_msgs/DangerousGoodsBasic.h>

namespace etsi_its_cam_conversion {
  
void convert_DangerousGoodsBasictoRos(const DangerousGoodsBasic_t& _DangerousGoodsBasic_in, etsi_its_cam_msgs::DangerousGoodsBasic& _DangerousGoodsBasic_out) {
  _DangerousGoodsBasic_out.value = _DangerousGoodsBasic_in;
}

void convert_DangerousGoodsBasictoC(const etsi_its_cam_msgs::DangerousGoodsBasic& _DangerousGoodsBasic_in, DangerousGoodsBasic_t& _DangerousGoodsBasic_out) {
  memset(&_DangerousGoodsBasic_out, 0, sizeof(DangerousGoodsBasic_t));
  _DangerousGoodsBasic_out = _DangerousGoodsBasic_in.value;
}

}