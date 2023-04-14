#pragma once

#include <etsi_its_cam_coding/ProtectedCommunicationZonesRSU.h>
#include <etsi_its_cam_msgs/ProtectedCommunicationZonesRSU.h>

namespace etsi_its_cam_conversion {
  
void toRos_ProtectedCommunicationZonesRSU(const ProtectedCommunicationZonesRSU_t& in, etsi_its_cam_msgs::ProtectedCommunicationZonesRSU& out) {
}

void toStruct_ProtectedCommunicationZonesRSU(const etsi_its_cam_msgs::ProtectedCommunicationZonesRSU& in, ProtectedCommunicationZonesRSU_t& out) {
  memset(&out, 0, sizeof(ProtectedCommunicationZonesRSU_t));
}

}