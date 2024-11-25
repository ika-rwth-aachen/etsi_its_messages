/** ============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University

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

#include <etsi_its_vam_ts_coding/vam_ts_EllipticalShape.h>
#include <etsi_its_vam_ts_conversion/convertCartesianPosition3d.h>
#include <etsi_its_vam_ts_conversion/convertStandardLength12b.h>
#include <etsi_its_vam_ts_conversion/convertWgs84AngleValue.h>
#ifdef ROS1
#include <etsi_its_vam_ts_msgs/EllipticalShape.h>
namespace vam_ts_msgs = etsi_its_vam_ts_msgs;
#else
#include <etsi_its_vam_ts_msgs/msg/elliptical_shape.hpp>
namespace vam_ts_msgs = etsi_its_vam_ts_msgs::msg;
#endif


namespace etsi_its_vam_ts_conversion {

void toRos_EllipticalShape(const vam_ts_EllipticalShape_t& in, vam_ts_msgs::EllipticalShape& out) {
  if (in.shapeReferencePoint) {
    toRos_CartesianPosition3d(*in.shapeReferencePoint, out.shape_reference_point);
    out.shape_reference_point_is_present = true;
  }
  toRos_StandardLength12b(in.semiMajorAxisLength, out.semi_major_axis_length);
  toRos_StandardLength12b(in.semiMinorAxisLength, out.semi_minor_axis_length);
  if (in.orientation) {
    toRos_Wgs84AngleValue(*in.orientation, out.orientation);
    out.orientation_is_present = true;
  }
  if (in.height) {
    toRos_StandardLength12b(*in.height, out.height);
    out.height_is_present = true;
  }
}

void toStruct_EllipticalShape(const vam_ts_msgs::EllipticalShape& in, vam_ts_EllipticalShape_t& out) {
  memset(&out, 0, sizeof(vam_ts_EllipticalShape_t));

  if (in.shape_reference_point_is_present) {
    out.shapeReferencePoint = (vam_ts_CartesianPosition3d_t*) calloc(1, sizeof(vam_ts_CartesianPosition3d_t));
    toStruct_CartesianPosition3d(in.shape_reference_point, *out.shapeReferencePoint);
  }
  toStruct_StandardLength12b(in.semi_major_axis_length, out.semiMajorAxisLength);
  toStruct_StandardLength12b(in.semi_minor_axis_length, out.semiMinorAxisLength);
  if (in.orientation_is_present) {
    out.orientation = (vam_ts_Wgs84AngleValue_t*) calloc(1, sizeof(vam_ts_Wgs84AngleValue_t));
    toStruct_Wgs84AngleValue(in.orientation, *out.orientation);
  }
  if (in.height_is_present) {
    out.height = (vam_ts_StandardLength12b_t*) calloc(1, sizeof(vam_ts_StandardLength12b_t));
    toStruct_StandardLength12b(in.height, *out.height);
  }
}

}
