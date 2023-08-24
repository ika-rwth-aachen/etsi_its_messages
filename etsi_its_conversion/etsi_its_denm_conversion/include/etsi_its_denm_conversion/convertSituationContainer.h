#pragma once

#include <etsi_its_denm_coding/SituationContainer.h>
#include <etsi_its_denm_conversion/convertInformationQuality.h>
#include <etsi_its_denm_conversion/convertCauseCode.h>
#include <etsi_its_denm_conversion/convertCauseCode.h>
#include <etsi_its_denm_conversion/convertEventHistory.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/situation_container.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/SituationContainer.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_SituationContainer(const SituationContainer_t& in, denm_msgs::SituationContainer& out) {

  toRos_InformationQuality(in.informationQuality, out.information_quality);
  toRos_CauseCode(in.eventType, out.event_type);
  if (in.linkedCause) {
    toRos_CauseCode(*in.linkedCause, out.linked_cause);
    out.linked_cause_is_present = true;
  }

  if (in.eventHistory) {
    toRos_EventHistory(*in.eventHistory, out.event_history);
    out.event_history_is_present = true;
  }

}

void toStruct_SituationContainer(const denm_msgs::SituationContainer& in, SituationContainer_t& out) {
    
  memset(&out, 0, sizeof(SituationContainer_t));

  toStruct_InformationQuality(in.information_quality, out.informationQuality);
  toStruct_CauseCode(in.event_type, out.eventType);
  if (in.linked_cause_is_present) {
    CauseCode_t linked_cause;
    toStruct_CauseCode(in.linked_cause, linked_cause);
    out.linkedCause = new CauseCode_t(linked_cause);
  }

  if (in.event_history_is_present) {
    EventHistory_t event_history;
    toStruct_EventHistory(in.event_history, event_history);
    out.eventHistory = new EventHistory_t(event_history);
  }

}

}