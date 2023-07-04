#pragma once

#include <etsi_its_cam_coding/PublicTransportContainer.h>
#include <etsi_its_cam_conversion/convertEmbarkationStatus.h>
#include <etsi_its_cam_conversion/convertPtActivation.h>
#include <etsi_its_cam_msgs/PublicTransportContainer.h>


namespace etsi_its_cam_conversion {

void toRos_PublicTransportContainer(const PublicTransportContainer_t& in, etsi_its_cam_msgs::PublicTransportContainer& out) {

  toRos_EmbarkationStatus(in.embarkationStatus, out.embarkationStatus);
  if (in.ptActivation) {
    toRos_PtActivation(*in.ptActivation, out.ptActivation);
    out.ptActivation_isPresent = true;
  }

}

void toStruct_PublicTransportContainer(const etsi_its_cam_msgs::PublicTransportContainer& in, PublicTransportContainer_t& out) {
    
  memset(&out, 0, sizeof(PublicTransportContainer_t));

  toStruct_EmbarkationStatus(in.embarkationStatus, out.embarkationStatus);
  if (in.ptActivation_isPresent) {
    PtActivation_t ptActivation;
    toStruct_PtActivation(in.ptActivation, ptActivation);
    out.ptActivation = new PtActivation_t(ptActivation);
  }

}

}