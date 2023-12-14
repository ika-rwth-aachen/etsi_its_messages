/** ============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

// --- Auto-generated by asn1ToConversionHeader.py -----------------------------

#pragma once

#include <etsi_its_spatem_coding/LaneTypeAttributes.h>
#include <etsi_its_spatem_conversion/convertLaneAttributesVehicle.h>
#include <etsi_its_spatem_conversion/convertLaneAttributesCrosswalk.h>
#include <etsi_its_spatem_conversion/convertLaneAttributesBike.h>
#include <etsi_its_spatem_conversion/convertLaneAttributesSidewalk.h>
#include <etsi_its_spatem_conversion/convertLaneAttributesBarrier.h>
#include <etsi_its_spatem_conversion/convertLaneAttributesStriping.h>
#include <etsi_its_spatem_conversion/convertLaneAttributesTrackedVehicle.h>
#include <etsi_its_spatem_conversion/convertLaneAttributesParking.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/LaneTypeAttributes.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/lane_type_attributes.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_LaneTypeAttributes(const LaneTypeAttributes_t& in, spatem_msgs::LaneTypeAttributes& out) {

  if (in.present == LaneTypeAttributes_PR::LaneTypeAttributes_PR_vehicle) {
    toRos_LaneAttributesVehicle(in.choice.vehicle, out.vehicle);
    out.choice = spatem_msgs::LaneTypeAttributes::CHOICE_VEHICLE;
  }

  if (in.present == LaneTypeAttributes_PR::LaneTypeAttributes_PR_crosswalk) {
    toRos_LaneAttributesCrosswalk(in.choice.crosswalk, out.crosswalk);
    out.choice = spatem_msgs::LaneTypeAttributes::CHOICE_CROSSWALK;
  }

  if (in.present == LaneTypeAttributes_PR::LaneTypeAttributes_PR_bikeLane) {
    toRos_LaneAttributesBike(in.choice.bikeLane, out.bike_lane);
    out.choice = spatem_msgs::LaneTypeAttributes::CHOICE_BIKE_LANE;
  }

  if (in.present == LaneTypeAttributes_PR::LaneTypeAttributes_PR_sidewalk) {
    toRos_LaneAttributesSidewalk(in.choice.sidewalk, out.sidewalk);
    out.choice = spatem_msgs::LaneTypeAttributes::CHOICE_SIDEWALK;
  }

  if (in.present == LaneTypeAttributes_PR::LaneTypeAttributes_PR_median) {
    toRos_LaneAttributesBarrier(in.choice.median, out.median);
    out.choice = spatem_msgs::LaneTypeAttributes::CHOICE_MEDIAN;
  }

  if (in.present == LaneTypeAttributes_PR::LaneTypeAttributes_PR_striping) {
    toRos_LaneAttributesStriping(in.choice.striping, out.striping);
    out.choice = spatem_msgs::LaneTypeAttributes::CHOICE_STRIPING;
  }

  if (in.present == LaneTypeAttributes_PR::LaneTypeAttributes_PR_trackedVehicle) {
    toRos_LaneAttributesTrackedVehicle(in.choice.trackedVehicle, out.tracked_vehicle);
    out.choice = spatem_msgs::LaneTypeAttributes::CHOICE_TRACKED_VEHICLE;
  }

  if (in.present == LaneTypeAttributes_PR::LaneTypeAttributes_PR_parking) {
    toRos_LaneAttributesParking(in.choice.parking, out.parking);
    out.choice = spatem_msgs::LaneTypeAttributes::CHOICE_PARKING;
  }
}

void toStruct_LaneTypeAttributes(const spatem_msgs::LaneTypeAttributes& in, LaneTypeAttributes_t& out) {

  memset(&out, 0, sizeof(LaneTypeAttributes_t));

  if (in.choice == spatem_msgs::LaneTypeAttributes::CHOICE_VEHICLE) {
    toStruct_LaneAttributesVehicle(in.vehicle, out.choice.vehicle);
    out.present = LaneTypeAttributes_PR::LaneTypeAttributes_PR_vehicle;
  }

  if (in.choice == spatem_msgs::LaneTypeAttributes::CHOICE_CROSSWALK) {
    toStruct_LaneAttributesCrosswalk(in.crosswalk, out.choice.crosswalk);
    out.present = LaneTypeAttributes_PR::LaneTypeAttributes_PR_crosswalk;
  }

  if (in.choice == spatem_msgs::LaneTypeAttributes::CHOICE_BIKE_LANE) {
    toStruct_LaneAttributesBike(in.bike_lane, out.choice.bikeLane);
    out.present = LaneTypeAttributes_PR::LaneTypeAttributes_PR_bikeLane;
  }

  if (in.choice == spatem_msgs::LaneTypeAttributes::CHOICE_SIDEWALK) {
    toStruct_LaneAttributesSidewalk(in.sidewalk, out.choice.sidewalk);
    out.present = LaneTypeAttributes_PR::LaneTypeAttributes_PR_sidewalk;
  }

  if (in.choice == spatem_msgs::LaneTypeAttributes::CHOICE_MEDIAN) {
    toStruct_LaneAttributesBarrier(in.median, out.choice.median);
    out.present = LaneTypeAttributes_PR::LaneTypeAttributes_PR_median;
  }

  if (in.choice == spatem_msgs::LaneTypeAttributes::CHOICE_STRIPING) {
    toStruct_LaneAttributesStriping(in.striping, out.choice.striping);
    out.present = LaneTypeAttributes_PR::LaneTypeAttributes_PR_striping;
  }

  if (in.choice == spatem_msgs::LaneTypeAttributes::CHOICE_TRACKED_VEHICLE) {
    toStruct_LaneAttributesTrackedVehicle(in.tracked_vehicle, out.choice.trackedVehicle);
    out.present = LaneTypeAttributes_PR::LaneTypeAttributes_PR_trackedVehicle;
  }

  if (in.choice == spatem_msgs::LaneTypeAttributes::CHOICE_PARKING) {
    toStruct_LaneAttributesParking(in.parking, out.choice.parking);
    out.present = LaneTypeAttributes_PR::LaneTypeAttributes_PR_parking;
  }

}

}