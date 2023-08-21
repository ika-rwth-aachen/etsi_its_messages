#pragma once

#include <etsi_its_cam_coding/SafetyCarContainer.h>
#include <etsi_its_cam_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_cam_conversion/convertCauseCode.h>
#include <etsi_its_cam_conversion/convertTrafficRule.h>
#include <etsi_its_cam_conversion/convertSpeedLimit.h>
#include <etsi_its_cam_msgs/SafetyCarContainer.h>


namespace etsi_its_cam_conversion {

void toRos_SafetyCarContainer(const SafetyCarContainer_t& in, etsi_its_cam_msgs::SafetyCarContainer& out) {

  toRos_LightBarSirenInUse(in.light_bar_siren_in_use, out.light_bar_siren_in_use);
  if (in.incident_indication) {
    toRos_CauseCode(*in.incident_indication, out.incident_indication);
    out.incident_indication_is_present = true;
  }

  if (in.traffic_rule) {
    toRos_TrafficRule(*in.traffic_rule, out.traffic_rule);
    out.traffic_rule_is_present = true;
  }

  if (in.speed_limit) {
    toRos_SpeedLimit(*in.speed_limit, out.speed_limit);
    out.speed_limit_is_present = true;
  }

}

void toStruct_SafetyCarContainer(const etsi_its_cam_msgs::SafetyCarContainer& in, SafetyCarContainer_t& out) {
    
  memset(&out, 0, sizeof(SafetyCarContainer_t));

  toStruct_LightBarSirenInUse(in.light_bar_siren_in_use, out.light_bar_siren_in_use);
  if (in.incident_indication_is_present) {
    CauseCode_t incident_indication;
    toStruct_CauseCode(in.incident_indication, incident_indication);
    out.incident_indication = new CauseCode_t(incident_indication);
  }

  if (in.traffic_rule_is_present) {
    TrafficRule_t traffic_rule;
    toStruct_TrafficRule(in.traffic_rule, traffic_rule);
    out.traffic_rule = new TrafficRule_t(traffic_rule);
  }

  if (in.speed_limit_is_present) {
    SpeedLimit_t speed_limit;
    toStruct_SpeedLimit(in.speed_limit, speed_limit);
    out.speed_limit = new SpeedLimit_t(speed_limit);
  }

}

}