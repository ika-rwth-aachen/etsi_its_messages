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

#include <etsi_its_spatem_coding/RequestorPositionVector.h>
#include <etsi_its_spatem_conversion/convertPosition3D.h>
#include <etsi_its_spatem_conversion/convertAngle.h>
#include <etsi_its_spatem_conversion/convertTransmissionAndSpeed.h>
#ifdef ROS1
#include <etsi_its_spatem_msgs/RequestorPositionVector.h>
namespace spatem_msgs = etsi_its_spatem_msgs;
#else
#include <etsi_its_spatem_msgs/msg/requestor_position_vector.hpp>
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
#endif


namespace etsi_its_spatem_conversion {

void toRos_RequestorPositionVector(const RequestorPositionVector_t& in, spatem_msgs::RequestorPositionVector& out) {

  toRos_Position3D(in.position, out.position);
  if (in.heading) {
    toRos_Angle(*in.heading, out.heading);
    out.heading_is_present = true;
  }

  if (in.speed) {
    toRos_TransmissionAndSpeed(*in.speed, out.speed);
    out.speed_is_present = true;
  }

}

void toStruct_RequestorPositionVector(const spatem_msgs::RequestorPositionVector& in, RequestorPositionVector_t& out) {

  memset(&out, 0, sizeof(RequestorPositionVector_t));

  toStruct_Position3D(in.position, out.position);
  if (in.heading_is_present) {
    Angle_t heading;
    toStruct_Angle(in.heading, heading);
    out.heading = new Angle_t(heading);
  }

  if (in.speed_is_present) {
    TransmissionAndSpeed_t speed;
    toStruct_TransmissionAndSpeed(in.speed, speed);
    out.speed = new TransmissionAndSpeed_t(speed);
  }

}

}