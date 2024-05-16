//// SEQUENCE RoadWorksContainerExtended


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/RoadWorksContainerExtended.h>
#include <etsi_its_denm_conversion/convertLightBarSirenInUse.h>
#include <etsi_its_denm_conversion/convertClosedLanes.h>
#include <etsi_its_denm_conversion/convertRestrictedTypes.h>
#include <etsi_its_denm_conversion/convertSpeedLimit.h>
#include <etsi_its_denm_conversion/convertCauseCode.h>
#include <etsi_its_denm_conversion/convertItineraryPath.h>
#include <etsi_its_denm_conversion/convertDeltaReferencePosition.h>
#include <etsi_its_denm_conversion/convertTrafficRule.h>
#include <etsi_its_denm_conversion/convertReferenceDenms.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/RoadWorksContainerExtended.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/road_works_container_extended.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_RoadWorksContainerExtended(const RoadWorksContainerExtended_t& in, denm_msgs::RoadWorksContainerExtended& out) {
  if (in.lightBarSirenInUse) {
    toRos_LightBarSirenInUse(*in.lightBarSirenInUse, out.light_bar_siren_in_use);
    out.light_bar_siren_in_use_is_present = true;
  }
  if (in.closedLanes) {
    toRos_ClosedLanes(*in.closedLanes, out.closed_lanes);
    out.closed_lanes_is_present = true;
  }
  if (in.restriction) {
    toRos_RestrictedTypes(*in.restriction, out.restriction);
    out.restriction_is_present = true;
  }
  if (in.speedLimit) {
    toRos_SpeedLimit(*in.speedLimit, out.speed_limit);
    out.speed_limit_is_present = true;
  }
  if (in.incidentIndication) {
    toRos_CauseCode(*in.incidentIndication, out.incident_indication);
    out.incident_indication_is_present = true;
  }
  if (in.recommendedPath) {
    toRos_ItineraryPath(*in.recommendedPath, out.recommended_path);
    out.recommended_path_is_present = true;
  }
  if (in.startingPointSpeedLimit) {
    toRos_DeltaReferencePosition(*in.startingPointSpeedLimit, out.starting_point_speed_limit);
    out.starting_point_speed_limit_is_present = true;
  }
  if (in.trafficFlowRule) {
    toRos_TrafficRule(*in.trafficFlowRule, out.traffic_flow_rule);
    out.traffic_flow_rule_is_present = true;
  }
  if (in.referenceDenms) {
    toRos_ReferenceDenms(*in.referenceDenms, out.reference_denms);
    out.reference_denms_is_present = true;
  }
}

void toStruct_RoadWorksContainerExtended(const denm_msgs::RoadWorksContainerExtended& in, RoadWorksContainerExtended_t& out) {
  memset(&out, 0, sizeof(RoadWorksContainerExtended_t));

  if (in.light_bar_siren_in_use_is_present) {
    out.lightBarSirenInUse = (LightBarSirenInUse_t*) calloc(1, sizeof(LightBarSirenInUse_t));
    toStruct_LightBarSirenInUse(in.light_bar_siren_in_use, *out.lightBarSirenInUse);
  }
  if (in.closed_lanes_is_present) {
    out.closedLanes = (ClosedLanes_t*) calloc(1, sizeof(ClosedLanes_t));
    toStruct_ClosedLanes(in.closed_lanes, *out.closedLanes);
  }
  if (in.restriction_is_present) {
    out.restriction = (RestrictedTypes_t*) calloc(1, sizeof(RestrictedTypes_t));
    toStruct_RestrictedTypes(in.restriction, *out.restriction);
  }
  if (in.speed_limit_is_present) {
    out.speedLimit = (SpeedLimit_t*) calloc(1, sizeof(SpeedLimit_t));
    toStruct_SpeedLimit(in.speed_limit, *out.speedLimit);
  }
  if (in.incident_indication_is_present) {
    out.incidentIndication = (CauseCode_t*) calloc(1, sizeof(CauseCode_t));
    toStruct_CauseCode(in.incident_indication, *out.incidentIndication);
  }
  if (in.recommended_path_is_present) {
    out.recommendedPath = (ItineraryPath_t*) calloc(1, sizeof(ItineraryPath_t));
    toStruct_ItineraryPath(in.recommended_path, *out.recommendedPath);
  }
  if (in.starting_point_speed_limit_is_present) {
    out.startingPointSpeedLimit = (DeltaReferencePosition_t*) calloc(1, sizeof(DeltaReferencePosition_t));
    toStruct_DeltaReferencePosition(in.starting_point_speed_limit, *out.startingPointSpeedLimit);
  }
  if (in.traffic_flow_rule_is_present) {
    out.trafficFlowRule = (TrafficRule_t*) calloc(1, sizeof(TrafficRule_t));
    toStruct_TrafficRule(in.traffic_flow_rule, *out.trafficFlowRule);
  }
  if (in.reference_denms_is_present) {
    out.referenceDenms = (ReferenceDenms_t*) calloc(1, sizeof(ReferenceDenms_t));
    toStruct_ReferenceDenms(in.reference_denms, *out.referenceDenms);
  }
}

}
