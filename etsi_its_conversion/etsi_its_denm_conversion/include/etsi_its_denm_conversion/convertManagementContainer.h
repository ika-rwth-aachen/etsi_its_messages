//// SEQUENCE ManagementContainer


#pragma once

#include <stdexcept>

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
#ifdef ROS1
#include <etsi_its_denm_msgs/ManagementContainer.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/management_container.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
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
  if (in.validityDuration) {
    toRos_ValidityDuration(*in.validityDuration, out.validity_duration);
  }
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
    out.termination = (Termination_t*) calloc(1, sizeof(Termination_t));
    toStruct_Termination(in.termination, *out.termination);
  }
  toStruct_ReferencePosition(in.event_position, out.eventPosition);
  if (in.relevance_distance_is_present) {
    out.relevanceDistance = (RelevanceDistance_t*) calloc(1, sizeof(RelevanceDistance_t));
    toStruct_RelevanceDistance(in.relevance_distance, *out.relevanceDistance);
  }
  if (in.relevance_traffic_direction_is_present) {
    out.relevanceTrafficDirection = (RelevanceTrafficDirection_t*) calloc(1, sizeof(RelevanceTrafficDirection_t));
    toStruct_RelevanceTrafficDirection(in.relevance_traffic_direction, *out.relevanceTrafficDirection);
  }
  out.validityDuration = (ValidityDuration_t*) calloc(1, sizeof(ValidityDuration_t));
  toStruct_ValidityDuration(in.validity_duration, *out.validityDuration);
  if (in.transmission_interval_is_present) {
    out.transmissionInterval = (TransmissionInterval_t*) calloc(1, sizeof(TransmissionInterval_t));
    toStruct_TransmissionInterval(in.transmission_interval, *out.transmissionInterval);
  }
  toStruct_StationType(in.station_type, out.stationType);
}

}
