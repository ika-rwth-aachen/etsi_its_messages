#pragma once

#include <etsi_its_cam_coding/DangerousGoodsBasic.h>
#include <etsi_its_cam_msgs/DangerousGoodsBasic.h>


namespace etsi_its_cam_conversion {

void toRos_DangerousGoodsBasic(const DangerousGoodsBasic_t& in, etsi_its_cam_msgs::DangerousGoodsBasic& out) {

  out.value = in;
}

void toStruct_DangerousGoodsBasic(const etsi_its_cam_msgs::DangerousGoodsBasic& in, DangerousGoodsBasic_t& out) {
    
  memset(&out, 0, sizeof(DangerousGoodsBasic_t));
  out = in.value;
}

}