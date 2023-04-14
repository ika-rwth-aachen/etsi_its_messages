#pragma once

#include <etsi_its_cam_coding/SafetyCarContainer.h>
#include <etsi_its_cam_msgs/SafetyCarContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertCauseCode.h>
#include <etsi_its_cam_conversion/convertTrafficRule.h>
#include <etsi_its_cam_conversion/convertSpeedLimit.h>

namespace etsi_its_cam_conversion {
  
void convert_SafetyCarContainertoRos(const SafetyCarContainer_t& _SafetyCarContainer_in, etsi_its_cam_msgs::SafetyCarContainer& _SafetyCarContainer_out) {
  convert_LightBarSirenInUsetoRos(_SafetyCarContainer_in.lightBarSirenInUse, _SafetyCarContainer_out.lightBarSirenInUse);
  if (_SafetyCarContainer_in.incidentIndication) {
    convert_CauseCodetoRos(*_SafetyCarContainer_in.incidentIndication, _SafetyCarContainer_out.incidentIndication);
    _SafetyCarContainer_out.incidentIndication_isPresent = true;
  }
  if (_SafetyCarContainer_in.trafficRule) {
    convert_TrafficRuletoRos(*_SafetyCarContainer_in.trafficRule, _SafetyCarContainer_out.trafficRule);
    _SafetyCarContainer_out.trafficRule_isPresent = true;
  }
  if (_SafetyCarContainer_in.speedLimit) {
    convert_SpeedLimittoRos(*_SafetyCarContainer_in.speedLimit, _SafetyCarContainer_out.speedLimit);
    _SafetyCarContainer_out.speedLimit_isPresent = true;
  }
}

void convert_SafetyCarContainertoC(const etsi_its_cam_msgs::SafetyCarContainer& _SafetyCarContainer_in, SafetyCarContainer_t& _SafetyCarContainer_out) {
  memset(&_SafetyCarContainer_out, 0, sizeof(SafetyCarContainer_t));
  convert_LightBarSirenInUsetoC(_SafetyCarContainer_in.lightBarSirenInUse, _SafetyCarContainer_out.lightBarSirenInUse);
  if (_SafetyCarContainer_in.incidentIndication_isPresent) {
    CauseCode_t incidentIndication;
    convert_CauseCodetoC(_SafetyCarContainer_in.incidentIndication, incidentIndication);
    _SafetyCarContainer_out.incidentIndication = new CauseCode_t(incidentIndication);
  }
  if (_SafetyCarContainer_in.trafficRule_isPresent) {
    TrafficRule_t trafficRule;
    convert_TrafficRuletoC(_SafetyCarContainer_in.trafficRule, trafficRule);
    _SafetyCarContainer_out.trafficRule = new TrafficRule_t(trafficRule);
  }
  if (_SafetyCarContainer_in.speedLimit_isPresent) {
    SpeedLimit_t speedLimit;
    convert_SpeedLimittoC(_SafetyCarContainer_in.speedLimit, speedLimit);
    _SafetyCarContainer_out.speedLimit = new SpeedLimit_t(speedLimit);
  }
}

}