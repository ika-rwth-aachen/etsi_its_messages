#pragma once

#include <etsi_its_cam_coding/EmergencyContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertCauseCode.h>
#include <etsi_its_cam_conversion/convertEmergencyPriority.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/EmergencyContainer.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/emergency_container.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_EmergencyContainer(const EmergencyContainer_t& in, cam_msgs::EmergencyContainer& out) {

  toRos_LightBarSirenInUse(in.lightBarSirenInUse, out.light_bar_siren_in_use);
  if (in.incidentIndication) {
    toRos_CauseCode(*in.incidentIndication, out.incident_indication);
    out.incident_indication_is_present = true;
  }

  if (in.emergencyPriority) {
    toRos_EmergencyPriority(*in.emergencyPriority, out.emergency_priority);
    out.emergency_priority_is_present = true;
  }

}

void toStruct_EmergencyContainer(const cam_msgs::EmergencyContainer& in, EmergencyContainer_t& out) {

  memset(&out, 0, sizeof(EmergencyContainer_t));

  toStruct_LightBarSirenInUse(in.light_bar_siren_in_use, out.lightBarSirenInUse);
  if (in.incident_indication_is_present) {
    CauseCode_t incident_indication;
    toStruct_CauseCode(in.incident_indication, incident_indication);
    out.incidentIndication = new CauseCode_t(incident_indication);
  }

  if (in.emergency_priority_is_present) {
    EmergencyPriority_t emergency_priority;
    toStruct_EmergencyPriority(in.emergency_priority, emergency_priority);
    out.emergencyPriority = new EmergencyPriority_t(emergency_priority);
  }

}

}