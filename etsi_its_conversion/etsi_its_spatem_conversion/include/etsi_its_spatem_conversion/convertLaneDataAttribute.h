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

#include <etsi_its_spatem_coding/LaneDataAttribute.h>
#include <etsi_its_spatem_conversion/convertDeltaAngle.h>
#include <etsi_its_spatem_conversion/convertRoadwayCrownAngle.h>
#include <etsi_its_spatem_conversion/convertRoadwayCrownAngle.h>
#include <etsi_its_spatem_conversion/convertRoadwayCrownAngle.h>
#include <etsi_its_spatem_conversion/convertMergeDivergeNodeAngle.h>
#include <etsi_its_spatem_conversion/convertSpeedLimitList.h>
#include <etsi_its_spatem_conversion/convertRegionalExtension[].h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/LaneDataAttribute.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/lane_data_attribute.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_LaneDataAttribute(const LaneDataAttribute_t& in, spatem_msgs::LaneDataAttribute& out) {

  if (in.present == LaneDataAttribute_PR::LaneDataAttribute_PR_pathEndPointAngle) {
    toRos_DeltaAngle(in.choice.pathEndPointAngle, out.path_end_point_angle);
    out.choice = spatem_msgs::LaneDataAttribute::CHOICE_PATH_END_POINT_ANGLE;
  }

  if (in.present == LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointCenter) {
    toRos_RoadwayCrownAngle(in.choice.laneCrownPointCenter, out.lane_crown_point_center);
    out.choice = spatem_msgs::LaneDataAttribute::CHOICE_LANE_CROWN_POINT_CENTER;
  }

  if (in.present == LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointLeft) {
    toRos_RoadwayCrownAngle(in.choice.laneCrownPointLeft, out.lane_crown_point_left);
    out.choice = spatem_msgs::LaneDataAttribute::CHOICE_LANE_CROWN_POINT_LEFT;
  }

  if (in.present == LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointRight) {
    toRos_RoadwayCrownAngle(in.choice.laneCrownPointRight, out.lane_crown_point_right);
    out.choice = spatem_msgs::LaneDataAttribute::CHOICE_LANE_CROWN_POINT_RIGHT;
  }

  if (in.present == LaneDataAttribute_PR::LaneDataAttribute_PR_laneAngle) {
    toRos_MergeDivergeNodeAngle(in.choice.laneAngle, out.lane_angle);
    out.choice = spatem_msgs::LaneDataAttribute::CHOICE_LANE_ANGLE;
  }

  if (in.present == LaneDataAttribute_PR::LaneDataAttribute_PR_speedLimits) {
    toRos_SpeedLimitList(in.choice.speedLimits, out.speed_limits);
    out.choice = spatem_msgs::LaneDataAttribute::CHOICE_SPEED_LIMITS;
  }

  if (in.present == LaneDataAttribute_PR::LaneDataAttribute_PR_regional) {
    toRos_RegionalExtension[](in.choice.regional, out.regional);
    out.choice = spatem_msgs::LaneDataAttribute::MIN_SIZE;
  }
  if (in.present == LaneDataAttribute_PR::LaneDataAttribute_PR_regional) {
    toRos_RegionalExtension[](in.choice.regional, out.regional);
    out.choice = spatem_msgs::LaneDataAttribute::MAX_SIZE;
  }
  if (in.present == LaneDataAttribute_PR::LaneDataAttribute_PR_regional) {
    toRos_RegionalExtension[](in.choice.regional, out.regional);
    out.choice = spatem_msgs::LaneDataAttribute::CHOICE_REGIONAL;
  }
}

void toStruct_LaneDataAttribute(const spatem_msgs::LaneDataAttribute& in, LaneDataAttribute_t& out) {

  memset(&out, 0, sizeof(LaneDataAttribute_t));

  if (in.choice == spatem_msgs::LaneDataAttribute::CHOICE_PATH_END_POINT_ANGLE) {
    toStruct_DeltaAngle(in.path_end_point_angle, out.choice.pathEndPointAngle);
    out.present = LaneDataAttribute_PR::LaneDataAttribute_PR_pathEndPointAngle;
  }

  if (in.choice == spatem_msgs::LaneDataAttribute::CHOICE_LANE_CROWN_POINT_CENTER) {
    toStruct_RoadwayCrownAngle(in.lane_crown_point_center, out.choice.laneCrownPointCenter);
    out.present = LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointCenter;
  }

  if (in.choice == spatem_msgs::LaneDataAttribute::CHOICE_LANE_CROWN_POINT_LEFT) {
    toStruct_RoadwayCrownAngle(in.lane_crown_point_left, out.choice.laneCrownPointLeft);
    out.present = LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointLeft;
  }

  if (in.choice == spatem_msgs::LaneDataAttribute::CHOICE_LANE_CROWN_POINT_RIGHT) {
    toStruct_RoadwayCrownAngle(in.lane_crown_point_right, out.choice.laneCrownPointRight);
    out.present = LaneDataAttribute_PR::LaneDataAttribute_PR_laneCrownPointRight;
  }

  if (in.choice == spatem_msgs::LaneDataAttribute::CHOICE_LANE_ANGLE) {
    toStruct_MergeDivergeNodeAngle(in.lane_angle, out.choice.laneAngle);
    out.present = LaneDataAttribute_PR::LaneDataAttribute_PR_laneAngle;
  }

  if (in.choice == spatem_msgs::LaneDataAttribute::CHOICE_SPEED_LIMITS) {
    toStruct_SpeedLimitList(in.speed_limits, out.choice.speedLimits);
    out.present = LaneDataAttribute_PR::LaneDataAttribute_PR_speedLimits;
  }

  if (in.choice == spatem_msgs::LaneDataAttribute::MIN_SIZE) {
    toStruct_RegionalExtension[](in.regional, out.choice.regional);
    out.present = LaneDataAttribute_PR::LaneDataAttribute_PR_regional;
  }
  if (in.choice == spatem_msgs::LaneDataAttribute::MAX_SIZE) {
    toStruct_RegionalExtension[](in.regional, out.choice.regional);
    out.present = LaneDataAttribute_PR::LaneDataAttribute_PR_regional;
  }
  if (in.choice == spatem_msgs::LaneDataAttribute::CHOICE_REGIONAL) {
    toStruct_RegionalExtension[](in.regional, out.choice.regional);
    out.present = LaneDataAttribute_PR::LaneDataAttribute_PR_regional;
  }

}

}