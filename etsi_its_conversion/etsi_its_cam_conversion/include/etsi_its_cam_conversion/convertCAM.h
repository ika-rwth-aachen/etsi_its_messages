#pragma once

#include <etsi_its_cam_coding/CAM.h>
#include <etsi_its_cam_msgs/CAM.h>
#include <etsi_its_cam_conversion/convertItsPduHeader.h>
#include <etsi_its_cam_conversion/convertCoopAwareness.h>

namespace etsi_its_cam_conversion {
  
void convert_CAMtoRos(const CAM_t& _CAM_in, etsi_its_cam_msgs::CAM& _CAM_out) {
  convert_ItsPduHeadertoRos(_CAM_in.header, _CAM_out.header);
  convert_CoopAwarenesstoRos(_CAM_in.cam, _CAM_out.cam);
}

void convert_CAMtoC(const etsi_its_cam_msgs::CAM& _CAM_in, CAM_t& _CAM_out) {
  memset(&_CAM_out, 0, sizeof(CAM_t));
  convert_ItsPduHeadertoC(_CAM_in.header, _CAM_out.header);
  convert_CoopAwarenesstoC(_CAM_in.cam, _CAM_out.cam);
}

}