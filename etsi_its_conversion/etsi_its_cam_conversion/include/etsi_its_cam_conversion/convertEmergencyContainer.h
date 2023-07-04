#pragma once

#include <etsi_its_cam_coding/EmergencyContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertCauseCode.h>
#include <etsi_its_cam_conversion/convertEmergencyPriority.h>
#include <etsi_its_cam_msgs/EmergencyContainer.h>


namespace etsi_its_cam_conversion {

void toRos_EmergencyContainer(const EmergencyContainer_t& in, etsi_its_cam_msgs::EmergencyContainer& out) {

  toRos_LightBarSirenInUse(in.lightBarSirenInUse, out.lightBarSirenInUse);
  if (in.incidentIndication) {
    toRos_CauseCode(*in.incidentIndication, out.incidentIndication);
    out.incidentIndication_isPresent = true;
  }

  if (in.emergencyPriority) {
    toRos_EmergencyPriority(*in.emergencyPriority, out.emergencyPriority);
    out.emergencyPriority_isPresent = true;
  }

}

void toStruct_EmergencyContainer(const etsi_its_cam_msgs::EmergencyContainer& in, EmergencyContainer_t& out) {
    
  memset(&out, 0, sizeof(EmergencyContainer_t));

  toStruct_LightBarSirenInUse(in.lightBarSirenInUse, out.lightBarSirenInUse);
  if (in.incidentIndication_isPresent) {
    CauseCode_t incidentIndication;
    toStruct_CauseCode(in.incidentIndication, incidentIndication);
    out.incidentIndication = new CauseCode_t(incidentIndication);
  }

  if (in.emergencyPriority_isPresent) {
    EmergencyPriority_t emergencyPriority;
    toStruct_EmergencyPriority(in.emergencyPriority, emergencyPriority);
    out.emergencyPriority = new EmergencyPriority_t(emergencyPriority);
  }

}

}