#pragma once

#include <etsi_its_cam_coding/RSUContainerHighFrequency.h>
#include <etsi_its_cam_conversion/convertProtectedCommunicationZonesRSU.h>
#include <etsi_its_cam_msgs/RSUContainerHighFrequency.h>


namespace etsi_its_cam_conversion {

void toRos_RSUContainerHighFrequency(const RSUContainerHighFrequency_t& in, etsi_its_cam_msgs::RSUContainerHighFrequency& out) {

  if (in.protected_communication_zones_r_s_u) {
    toRos_ProtectedCommunicationZonesRSU(*in.protected_communication_zones_r_s_u, out.protected_communication_zones_r_s_u);
    out.protected_communication_zones_r_s_u_is_present = true;
  }

}

void toStruct_RSUContainerHighFrequency(const etsi_its_cam_msgs::RSUContainerHighFrequency& in, RSUContainerHighFrequency_t& out) {
    
  memset(&out, 0, sizeof(RSUContainerHighFrequency_t));

  if (in.protected_communication_zones_r_s_u_is_present) {
    ProtectedCommunicationZonesRSU_t protected_communication_zones_r_s_u;
    toStruct_ProtectedCommunicationZonesRSU(in.protected_communication_zones_r_s_u, protected_communication_zones_r_s_u);
    out.protected_communication_zones_r_s_u = new ProtectedCommunicationZonesRSU_t(protected_communication_zones_r_s_u);
  }

}

}