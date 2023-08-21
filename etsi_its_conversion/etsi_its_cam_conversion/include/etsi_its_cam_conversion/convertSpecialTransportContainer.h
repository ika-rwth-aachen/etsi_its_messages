#pragma once

#include <etsi_its_cam_coding/SpecialTransportContainer.h>
#include <etsi_its_cam_conversion/convertSpecialTransportType.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_msgs/SpecialTransportContainer.h>


namespace etsi_its_cam_conversion {

void toRos_SpecialTransportContainer(const SpecialTransportContainer_t& in, etsi_its_cam_msgs::SpecialTransportContainer& out) {

  toRos_SpecialTransportType(in.specialTransportType, out.special_transport_type);
  toRos_LightBarSirenInUse(in.lightBarSirenInUse, out.light_bar_siren_in_use);
}

void toStruct_SpecialTransportContainer(const etsi_its_cam_msgs::SpecialTransportContainer& in, SpecialTransportContainer_t& out) {
    
  memset(&out, 0, sizeof(SpecialTransportContainer_t));

  toStruct_SpecialTransportType(in.special_transport_type, out.specialTransportType);
  toStruct_LightBarSirenInUse(in.light_bar_siren_in_use, out.lightBarSirenInUse);
}

}