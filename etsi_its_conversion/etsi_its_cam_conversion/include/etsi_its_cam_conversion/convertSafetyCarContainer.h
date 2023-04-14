#pragma once

#include <etsi_its_cam_coding/SafetyCarContainer.h>
#include <etsi_its_cam_msgs/SafetyCarContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertCauseCode.h>
#include <etsi_its_cam_conversion/convertTrafficRule.h>
#include <etsi_its_cam_conversion/convertSpeedLimit.h>

namespace etsi_its_cam_conversion {
  
void toRos_SafetyCarContainer(const SafetyCarContainer_t& in, etsi_its_cam_msgs::SafetyCarContainer& out) {
  toRos_LightBarSirenInUse(in.lightBarSirenInUse, out.lightBarSirenInUse);
  if (in.incidentIndication) {
    toRos_CauseCode(*in.incidentIndication, out.incidentIndication);
    out.incidentIndication_isPresent = true;
  }
  if (in.trafficRule) {
    toRos_TrafficRule(*in.trafficRule, out.trafficRule);
    out.trafficRule_isPresent = true;
  }
  if (in.speedLimit) {
    toRos_SpeedLimit(*in.speedLimit, out.speedLimit);
    out.speedLimit_isPresent = true;
  }
}

void toStruct_SafetyCarContainer(const etsi_its_cam_msgs::SafetyCarContainer& in, SafetyCarContainer_t& out) {
  memset(&out, 0, sizeof(SafetyCarContainer_t));
  toStruct_LightBarSirenInUse(in.lightBarSirenInUse, out.lightBarSirenInUse);
  if (in.incidentIndication_isPresent) {
    CauseCode_t incidentIndication;
    toStruct_CauseCode(in.incidentIndication, incidentIndication);
    out.incidentIndication = new CauseCode_t(incidentIndication);
  }
  if (in.trafficRule_isPresent) {
    TrafficRule_t trafficRule;
    toStruct_TrafficRule(in.trafficRule, trafficRule);
    out.trafficRule = new TrafficRule_t(trafficRule);
  }
  if (in.speedLimit_isPresent) {
    SpeedLimit_t speedLimit;
    toStruct_SpeedLimit(in.speedLimit, speedLimit);
    out.speedLimit = new SpeedLimit_t(speedLimit);
  }
}

}