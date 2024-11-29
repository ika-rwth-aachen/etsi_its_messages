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

#include <etsi_its_cpm_ts_coding/cpm_ts_CircularShape.h>
#include <etsi_its_cpm_ts_conversion/convertCartesianPosition3d.h>
#include <etsi_its_cpm_ts_conversion/convertStandardLength12b.h>
#ifdef ROS1
#include <etsi_its_cpm_ts_msgs/CircularShape.h>
namespace cpm_ts_msgs = etsi_its_cpm_ts_msgs;
#else
#include <etsi_its_cpm_ts_msgs/msg/circular_shape.hpp>
namespace cpm_ts_msgs = etsi_its_cpm_ts_msgs::msg;
#endif


namespace etsi_its_cpm_ts_conversion {

void toRos_CircularShape(const cpm_ts_CircularShape_t& in, cpm_ts_msgs::CircularShape& out) {
  if (in.shapeReferencePoint) {
    toRos_CartesianPosition3d(*in.shapeReferencePoint, out.shape_reference_point);
    out.shape_reference_point_is_present = true;
  }
  toRos_StandardLength12b(in.radius, out.radius);
  if (in.height) {
    toRos_StandardLength12b(*in.height, out.height);
    out.height_is_present = true;
  }
}

void toStruct_CircularShape(const cpm_ts_msgs::CircularShape& in, cpm_ts_CircularShape_t& out) {
  memset(&out, 0, sizeof(cpm_ts_CircularShape_t));

  if (in.shape_reference_point_is_present) {
    out.shapeReferencePoint = (cpm_ts_CartesianPosition3d_t*) calloc(1, sizeof(cpm_ts_CartesianPosition3d_t));
    toStruct_CartesianPosition3d(in.shape_reference_point, *out.shapeReferencePoint);
  }
  toStruct_StandardLength12b(in.radius, out.radius);
  if (in.height_is_present) {
    out.height = (cpm_ts_StandardLength12b_t*) calloc(1, sizeof(cpm_ts_StandardLength12b_t));
    toStruct_StandardLength12b(in.height, *out.height);
  }
}

}
