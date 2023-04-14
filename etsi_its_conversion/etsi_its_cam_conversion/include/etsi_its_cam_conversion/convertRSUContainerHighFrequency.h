#pragma once

#include <etsi_its_cam_coding/RSUContainerHighFrequency.h>
#include <etsi_its_cam_msgs/RSUContainerHighFrequency.h>
#include <etsi_its_cam_conversion/convertProtectedCommunicationZonesRSU.h>

namespace etsi_its_cam_conversion {
  
void toRos_RSUContainerHighFrequency(const RSUContainerHighFrequency_t& in, etsi_its_cam_msgs::RSUContainerHighFrequency& out) {
  if (in.protectedCommunicationZonesRSU) {
    toRos_ProtectedCommunicationZonesRSU(*in.protectedCommunicationZonesRSU, out.protectedCommunicationZonesRSU);
    out.protectedCommunicationZonesRSU_isPresent = true;
  }
}

void toStruct_RSUContainerHighFrequency(const etsi_its_cam_msgs::RSUContainerHighFrequency& in, RSUContainerHighFrequency_t& out) {
  memset(&out, 0, sizeof(RSUContainerHighFrequency_t));
  if (in.protectedCommunicationZonesRSU_isPresent) {
    ProtectedCommunicationZonesRSU_t protectedCommunicationZonesRSU;
    toStruct_ProtectedCommunicationZonesRSU(in.protectedCommunicationZonesRSU, protectedCommunicationZonesRSU);
    out.protectedCommunicationZonesRSU = new ProtectedCommunicationZonesRSU_t(protectedCommunicationZonesRSU);
  }
}

}