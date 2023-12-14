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

#include <etsi_its_spatem_coding/RestrictionUserType.h>
#include <etsi_its_spatem_conversion/convertRestrictionAppliesTo.h>
#include <etsi_its_spatem_conversion/convertRegionalExtension.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/RestrictionUserType.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/restriction_user_type.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_RestrictionUserType(const RestrictionUserType_t& in, spatem_msgs::RestrictionUserType& out) {

  if (in.present == RestrictionUserType_PR::RestrictionUserType_PR_basicType) {
    toRos_RestrictionAppliesTo(in.choice.basicType, out.basic_type);
    out.choice = spatem_msgs::RestrictionUserType::CHOICE_BASIC_TYPE;
  }

  if (in.present == RestrictionUserType_PR::RestrictionUserType_PR_regional) {
    // TODO: toRos_RegionalExtension(in.choice.regional, out.regional);
    out.choice = spatem_msgs::RestrictionUserType::REGIONAL_MIN_SIZE;
  }
  if (in.present == RestrictionUserType_PR::RestrictionUserType_PR_regional) {
    // TODO: toRos_RegionalExtension(in.choice.regional, out.regional);
    out.choice = spatem_msgs::RestrictionUserType::REGIONAL_MAX_SIZE;
  }
  if (in.present == RestrictionUserType_PR::RestrictionUserType_PR_regional) {
    // TODO: toRos_RegionalExtension(in.choice.regional, out.regional);
    out.choice = spatem_msgs::RestrictionUserType::CHOICE_REGIONAL;
  }
}

void toStruct_RestrictionUserType(const spatem_msgs::RestrictionUserType& in, RestrictionUserType_t& out) {

  memset(&out, 0, sizeof(RestrictionUserType_t));

  if (in.choice == spatem_msgs::RestrictionUserType::CHOICE_BASIC_TYPE) {
    toStruct_RestrictionAppliesTo(in.basic_type, out.choice.basicType);
    out.present = RestrictionUserType_PR::RestrictionUserType_PR_basicType;
  }

  if (in.choice == spatem_msgs::RestrictionUserType::REGIONAL_MIN_SIZE) {
    // TODO: toStruct_RegionalExtension(in.regional, out.choice.regional);
    out.present = RestrictionUserType_PR::RestrictionUserType_PR_regional;
  }
  if (in.choice == spatem_msgs::RestrictionUserType::REGIONAL_MAX_SIZE) {
    // TODO: toStruct_RegionalExtension(in.regional, out.choice.regional);
    out.present = RestrictionUserType_PR::RestrictionUserType_PR_regional;
  }
  if (in.choice == spatem_msgs::RestrictionUserType::CHOICE_REGIONAL) {
    // TODO: toStruct_RegionalExtension(in.regional, out.choice.regional);
    out.present = RestrictionUserType_PR::RestrictionUserType_PR_regional;
  }

}

}