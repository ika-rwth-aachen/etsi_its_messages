#pragma once

#include <etsi_its_cam_coding/RSUContainerHighFrequency.h>
#include <etsi_its_cam_msgs/RSUContainerHighFrequency.h>
#include <etsi_its_cam_conversion/convertProtectedCommunicationZonesRSU.h>

namespace etsi_its_cam_conversion {
  
void convert_RSUContainerHighFrequencytoRos(const RSUContainerHighFrequency_t& _RSUContainerHighFrequency_in, etsi_its_cam_msgs::RSUContainerHighFrequency& _RSUContainerHighFrequency_out) {
  if (_RSUContainerHighFrequency_in.protectedCommunicationZonesRSU) {
    convert_ProtectedCommunicationZonesRSUtoRos(*_RSUContainerHighFrequency_in.protectedCommunicationZonesRSU, _RSUContainerHighFrequency_out.protectedCommunicationZonesRSU);
    _RSUContainerHighFrequency_out.protectedCommunicationZonesRSU_isPresent = true;
  }
}

void convert_RSUContainerHighFrequencytoC(const etsi_its_cam_msgs::RSUContainerHighFrequency& _RSUContainerHighFrequency_in, RSUContainerHighFrequency_t& _RSUContainerHighFrequency_out) {
  memset(&_RSUContainerHighFrequency_out, 0, sizeof(RSUContainerHighFrequency_t));
  if (_RSUContainerHighFrequency_in.protectedCommunicationZonesRSU_isPresent) {
    ProtectedCommunicationZonesRSU_t protectedCommunicationZonesRSU;
    convert_ProtectedCommunicationZonesRSUtoC(_RSUContainerHighFrequency_in.protectedCommunicationZonesRSU, protectedCommunicationZonesRSU);
    _RSUContainerHighFrequency_out.protectedCommunicationZonesRSU = new ProtectedCommunicationZonesRSU_t(protectedCommunicationZonesRSU);
  }
}

}