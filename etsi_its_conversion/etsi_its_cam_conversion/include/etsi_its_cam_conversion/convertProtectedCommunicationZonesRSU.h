#pragma once

#include <etsi_its_cam_coding/ProtectedCommunicationZonesRSU.h>
#include <etsi_its_cam_msgs/ProtectedCommunicationZonesRSU.h>
#include <etsi_its_cam_coding/ProtectedCommunicationZone.h>
#include <etsi_its_cam_msgs/ProtectedCommunicationZone.h>
#include <etsi_its_cam_conversion/convertProtectedCommunicationZone.h>

#include <stdexcept>
#include <etsi_its_cam_coding/asn_SEQUENCE_OF.h>

namespace etsi_its_cam_conversion {
  
void toRos_ProtectedCommunicationZonesRSU(const ProtectedCommunicationZonesRSU_t& in, etsi_its_cam_msgs::ProtectedCommunicationZonesRSU& out) {
  for (int i = 0; i < in.list.count; i++) {
    etsi_its_cam_msgs::ProtectedCommunicationZone protectedCommunicationZone;
    toRos_ProtectedCommunicationZone(*(in.list.array[i]), protectedCommunicationZone);
    out.array.push_back(protectedCommunicationZone);
  }
}

void toStruct_ProtectedCommunicationZonesRSU(const etsi_its_cam_msgs::ProtectedCommunicationZonesRSU& in, ProtectedCommunicationZonesRSU_t& out) {
  memset(&out, 0, sizeof(ProtectedCommunicationZonesRSU_t));
  for (int i = 0; i < in.array.size(); i++) {
    ProtectedCommunicationZone_t protectedCommunicationZone;
    toStruct_ProtectedCommunicationZone(in.array[i], protectedCommunicationZone);
    ProtectedCommunicationZone_t* protectedCommunicationZone_ptr = new ProtectedCommunicationZone_t(protectedCommunicationZone);
    int status = asn_sequence_add(&out, protectedCommunicationZone_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}