#pragma once

#include <etsi_its_cam_coding/DangerousGoodsContainer.h>
#include <etsi_its_cam_conversion/convertDangerousGoodsBasic.h>
#include <etsi_its_cam_msgs/DangerousGoodsContainer.h>


namespace etsi_its_cam_conversion {

void toRos_DangerousGoodsContainer(const DangerousGoodsContainer_t& in, etsi_its_cam_msgs::DangerousGoodsContainer& out) {

  toRos_DangerousGoodsBasic(in.dangerousGoodsBasic, out.dangerous_goods_basic);
}

void toStruct_DangerousGoodsContainer(const etsi_its_cam_msgs::DangerousGoodsContainer& in, DangerousGoodsContainer_t& out) {
    
  memset(&out, 0, sizeof(DangerousGoodsContainer_t));

  toStruct_DangerousGoodsBasic(in.dangerous_goods_basic, out.dangerousGoodsBasic);
}

}