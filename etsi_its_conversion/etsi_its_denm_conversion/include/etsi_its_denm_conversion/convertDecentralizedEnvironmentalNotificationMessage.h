#pragma once

#include <etsi_its_denm_coding/DecentralizedEnvironmentalNotificationMessage.h>
#include <etsi_its_denm_conversion/convertManagementContainer.h>
#include <etsi_its_denm_conversion/convertSituationContainer.h>
#include <etsi_its_denm_conversion/convertLocationContainer.h>
#include <etsi_its_denm_conversion/convertAlacarteContainer.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/decentralized_environmental_notification_message.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/DecentralizedEnvironmentalNotificationMessage.h>
namespace denm_msgs = etsi_its_denm_msgs;
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
    SituationContainer_t situation;
    toStruct_SituationContainer(in.situation, situation);
    out.situation = new SituationContainer_t(situation);
  }

  if (in.location_is_present) {
    LocationContainer_t location;
    toStruct_LocationContainer(in.location, location);
    out.location = new LocationContainer_t(location);
  }

  if (in.alacarte_is_present) {
    AlacarteContainer_t alacarte;
    toStruct_AlacarteContainer(in.alacarte, alacarte);
    out.alacarte = new AlacarteContainer_t(alacarte);
  }

}

}