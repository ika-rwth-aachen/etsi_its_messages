#pragma once

#include <etsi_its_cam_coding/PublicTransportContainer.h>
#include <etsi_its_cam_conversion/convertEmbarkationStatus.h>
#include <etsi_its_cam_conversion/convertPtActivation.h>
#include <etsi_its_cam_msgs/PublicTransportContainer.h>


namespace etsi_its_cam_conversion {

void toRos_PublicTransportContainer(const PublicTransportContainer_t& in, etsi_its_cam_msgs::PublicTransportContainer& out) {

  toRos_EmbarkationStatus(in.embarkationStatus, out.embarkation_status);
  if (in.ptActivation) {
    toRos_PtActivation(*in.ptActivation, out.pt_activation);
    out.pt_activation_is_present = true;
  }

}

void toStruct_PublicTransportContainer(const etsi_its_cam_msgs::PublicTransportContainer& in, PublicTransportContainer_t& out) {
    
  memset(&out, 0, sizeof(PublicTransportContainer_t));

  toStruct_EmbarkationStatus(in.embarkation_status, out.embarkationStatus);
  if (in.pt_activation_is_present) {
    PtActivation_t pt_activation;
    toStruct_PtActivation(in.pt_activation, pt_activation);
    out.ptActivation = new PtActivation_t(pt_activation);
  }

}

}