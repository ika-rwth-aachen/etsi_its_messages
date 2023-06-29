#pragma once

#include <etsi_its_cam_coding/EmbarkationStatus.h>
#include <etsi_its_cam_conversion/primitives/convertBOOLEAN.h>
#include <etsi_its_cam_msgs/EmbarkationStatus.h>


namespace etsi_its_cam_conversion {

void toRos_EmbarkationStatus(const EmbarkationStatus_t& in, etsi_its_cam_msgs::EmbarkationStatus& out) {

  toRos_BOOLEAN(in, out.value);
}

void toStruct_EmbarkationStatus(const etsi_its_cam_msgs::EmbarkationStatus& in, EmbarkationStatus_t& out) {
    
  memset(&out, 0, sizeof(EmbarkationStatus_t));
  toStruct_BOOLEAN(in.value, out);
}

}