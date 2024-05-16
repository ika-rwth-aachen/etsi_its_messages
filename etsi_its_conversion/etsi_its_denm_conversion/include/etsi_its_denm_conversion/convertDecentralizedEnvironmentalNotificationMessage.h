//// SEQUENCE DecentralizedEnvironmentalNotificationMessage


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/DecentralizedEnvironmentalNotificationMessage.h>
#include <etsi_its_denm_conversion/convertManagementContainer.h>
#include <etsi_its_denm_conversion/convertSituationContainer.h>
#include <etsi_its_denm_conversion/convertLocationContainer.h>
#include <etsi_its_denm_conversion/convertAlacarteContainer.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/DecentralizedEnvironmentalNotificationMessage.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/decentralized_environmental_notification_message.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_DecentralizedEnvironmentalNotificationMessage(const DecentralizedEnvironmentalNotificationMessage_t& in, denm_msgs::DecentralizedEnvironmentalNotificationMessage& out) {
  toRos_ManagementContainer(in.management, out.management);
  if (in.situation) {
    toRos_SituationContainer(*in.situation, out.situation);
    out.situation_is_present = true;
  }
  if (in.location) {
    toRos_LocationContainer(*in.location, out.location);
    out.location_is_present = true;
  }
  if (in.alacarte) {
    toRos_AlacarteContainer(*in.alacarte, out.alacarte);
    out.alacarte_is_present = true;
  }
}

void toStruct_DecentralizedEnvironmentalNotificationMessage(const denm_msgs::DecentralizedEnvironmentalNotificationMessage& in, DecentralizedEnvironmentalNotificationMessage_t& out) {
  memset(&out, 0, sizeof(DecentralizedEnvironmentalNotificationMessage_t));

  toStruct_ManagementContainer(in.management, out.management);
  if (in.situation_is_present) {
    out.situation = (SituationContainer_t*) calloc(1, sizeof(SituationContainer_t));
    toStruct_SituationContainer(in.situation, *out.situation);
  }
  if (in.location_is_present) {
    out.location = (LocationContainer_t*) calloc(1, sizeof(LocationContainer_t));
    toStruct_LocationContainer(in.location, *out.location);
  }
  if (in.alacarte_is_present) {
    out.alacarte = (AlacarteContainer_t*) calloc(1, sizeof(AlacarteContainer_t));
    toStruct_AlacarteContainer(in.alacarte, *out.alacarte);
  }
}

}
