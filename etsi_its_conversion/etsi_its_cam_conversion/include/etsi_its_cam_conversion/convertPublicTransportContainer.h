#pragma once

#include <etsi_its_cam_coding/PublicTransportContainer.h>
#include <etsi_its_cam_msgs/PublicTransportContainer.h>
#include <etsi_its_cam_conversion/convertEmbarkationStatus.h>
#include <etsi_its_cam_conversion/convertPtActivation.h>

namespace etsi_its_cam_conversion {
  
void convert_PublicTransportContainertoRos(const PublicTransportContainer_t& _PublicTransportContainer_in, etsi_its_cam_msgs::PublicTransportContainer& _PublicTransportContainer_out) {
  convert_EmbarkationStatustoRos(_PublicTransportContainer_in.embarkationStatus, _PublicTransportContainer_out.embarkationStatus);
  if (_PublicTransportContainer_in.ptActivation) {
    convert_PtActivationtoRos(*_PublicTransportContainer_in.ptActivation, _PublicTransportContainer_out.ptActivation);
    _PublicTransportContainer_out.ptActivation_isPresent = true;
  }
}

void convert_PublicTransportContainertoC(const etsi_its_cam_msgs::PublicTransportContainer& _PublicTransportContainer_in, PublicTransportContainer_t& _PublicTransportContainer_out) {
  memset(&_PublicTransportContainer_out, 0, sizeof(PublicTransportContainer_t));
  convert_EmbarkationStatustoC(_PublicTransportContainer_in.embarkationStatus, _PublicTransportContainer_out.embarkationStatus);
  if (_PublicTransportContainer_in.ptActivation_isPresent) {
    PtActivation_t ptActivation;
    convert_PtActivationtoC(_PublicTransportContainer_in.ptActivation, ptActivation);
    _PublicTransportContainer_out.ptActivation = new PtActivation_t(ptActivation);
  }
}

}