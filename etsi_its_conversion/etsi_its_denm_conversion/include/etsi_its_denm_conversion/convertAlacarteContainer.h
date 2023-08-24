#pragma once

#include <etsi_its_denm_coding/AlacarteContainer.h>
#include <etsi_its_denm_conversion/convertLanePosition.h>
#include <etsi_its_denm_conversion/convertImpactReductionContainer.h>
#include <etsi_its_denm_conversion/convertTemperature.h>
#include <etsi_its_denm_conversion/convertRoadWorksContainerExtended.h>
#include <etsi_its_denm_conversion/convertPositioningSolutionType.h>
#include <etsi_its_denm_conversion/convertStationaryVehicleContainer.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/alacarte_container.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/AlacarteContainer.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_AlacarteContainer(const AlacarteContainer_t& in, denm_msgs::AlacarteContainer& out) {

  if (in.lanePosition) {
    toRos_LanePosition(*in.lanePosition, out.lane_position);
    out.lane_position_is_present = true;
  }

  if (in.impactReduction) {
    toRos_ImpactReductionContainer(*in.impactReduction, out.impact_reduction);
    out.impact_reduction_is_present = true;
  }

  if (in.externalTemperature) {
    toRos_Temperature(*in.externalTemperature, out.external_temperature);
    out.external_temperature_is_present = true;
  }

  if (in.roadWorks) {
    toRos_RoadWorksContainerExtended(*in.roadWorks, out.road_works);
    out.road_works_is_present = true;
  }

  if (in.positioningSolution) {
    toRos_PositioningSolutionType(*in.positioningSolution, out.positioning_solution);
    out.positioning_solution_is_present = true;
  }

  if (in.stationaryVehicle) {
    toRos_StationaryVehicleContainer(*in.stationaryVehicle, out.stationary_vehicle);
    out.stationary_vehicle_is_present = true;
  }

}

void toStruct_AlacarteContainer(const denm_msgs::AlacarteContainer& in, AlacarteContainer_t& out) {
    
  memset(&out, 0, sizeof(AlacarteContainer_t));

  if (in.lane_position_is_present) {
    LanePosition_t lane_position;
    toStruct_LanePosition(in.lane_position, lane_position);
    out.lanePosition = new LanePosition_t(lane_position);
  }

  if (in.impact_reduction_is_present) {
    ImpactReductionContainer_t impact_reduction;
    toStruct_ImpactReductionContainer(in.impact_reduction, impact_reduction);
    out.impactReduction = new ImpactReductionContainer_t(impact_reduction);
  }

  if (in.external_temperature_is_present) {
    Temperature_t external_temperature;
    toStruct_Temperature(in.external_temperature, external_temperature);
    out.externalTemperature = new Temperature_t(external_temperature);
  }

  if (in.road_works_is_present) {
    RoadWorksContainerExtended_t road_works;
    toStruct_RoadWorksContainerExtended(in.road_works, road_works);
    out.roadWorks = new RoadWorksContainerExtended_t(road_works);
  }

  if (in.positioning_solution_is_present) {
    PositioningSolutionType_t positioning_solution;
    toStruct_PositioningSolutionType(in.positioning_solution, positioning_solution);
    out.positioningSolution = new PositioningSolutionType_t(positioning_solution);
  }

  if (in.stationary_vehicle_is_present) {
    StationaryVehicleContainer_t stationary_vehicle;
    toStruct_StationaryVehicleContainer(in.stationary_vehicle, stationary_vehicle);
    out.stationaryVehicle = new StationaryVehicleContainer_t(stationary_vehicle);
  }

}

}