#pragma once

#include <etsi_its_denm_coding/ManagementContainer.h>
#include <etsi_its_denm_conversion/convertActionID.h>
#include <etsi_its_denm_conversion/convertTimestampIts.h>
#include <etsi_its_denm_conversion/convertTimestampIts.h>
#include <etsi_its_denm_conversion/convertTermination.h>
#include <etsi_its_denm_conversion/convertReferencePosition.h>
#include <etsi_its_denm_conversion/convertRelevanceDistance.h>
#include <etsi_its_denm_conversion/convertRelevanceTrafficDirection.h>
#include <etsi_its_denm_conversion/convertValidityDuration.h>
#include <etsi_its_denm_conversion/convertTransmissionInterval.h>
#include <etsi_its_denm_conversion/convertStationType.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/management_container.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/ManagementContainer.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_ManagementContainer(const ManagementContainer_t& in, denm_msgs::ManagementContainer& out) {

  toRos_ActionID(in.actionID, out.action_id);
  toRos_TimestampIts(in.detectionTime, out.detection_time);
  toRos_TimestampIts(in.referenceTime, out.reference_time);
  if (in.termination) {
    toRos_Termination(*in.termination, out.termination);
    out.termination_is_present = true;
  }

  toRos_ReferencePosition(in.eventPosition, out.event_position);
  if (in.relevanceDistance) {
    toRos_RelevanceDistance(*in.relevanceDistance, out.relevance_distance);
    out.relevance_distance_is_present = true;
  }

  if (in.relevanceTrafficDirection) {
    toRos_RelevanceTrafficDirection(*in.relevanceTrafficDirection, out.relevance_traffic_direction);
    out.relevance_traffic_direction_is_present = true;
  }

  toRos_ValidityDuration(in.validityDuration, out.validity_duration);
  if (in.transmissionInterval) {
    toRos_TransmissionInterval(*in.transmissionInterval, out.transmission_interval);
    out.transmission_interval_is_present = true;
  }

  toRos_StationType(in.stationType, out.station_type);
}

void toStruct_ManagementContainer(const denm_msgs::ManagementContainer& in, ManagementContainer_t& out) {

  memset(&out, 0, sizeof(ManagementContainer_t));

  toStruct_ActionID(in.action_id, out.actionID);
  toStruct_TimestampIts(in.detection_time, out.detectionTime);
  toStruct_TimestampIts(in.reference_time, out.referenceTime);
  if (in.termination_is_present) {
    Termination_t termination;
    toStruct_Termination(in.termination, termination);
    out.termination = new Termination_t(termination);
  }

  toStruct_ReferencePosition(in.event_position, out.eventPosition);
  if (in.relevance_distance_is_present) {
    RelevanceDistance_t relevance_distance;
    toStruct_RelevanceDistance(in.relevance_distance, relevance_distance);
    out.relevanceDistance = new RelevanceDistance_t(relevance_distance);
  }

  if (in.relevance_traffic_direction_is_present) {
    RelevanceTrafficDirection_t relevance_traffic_direction;
    toStruct_RelevanceTrafficDirection(in.relevance_traffic_direction, relevance_traffic_direction);
    out.relevanceTrafficDirection = new RelevanceTrafficDirection_t(relevance_traffic_direction);
  }

  toStruct_ValidityDuration(in.validity_duration, out.validityDuration);
  if (in.transmission_interval_is_present) {
    TransmissionInterval_t transmission_interval;
    toStruct_TransmissionInterval(in.transmission_interval, transmission_interval);
    out.transmissionInterval = new TransmissionInterval_t(transmission_interval);
  }

  toStruct_StationType(in.station_type, out.stationType);
}

}