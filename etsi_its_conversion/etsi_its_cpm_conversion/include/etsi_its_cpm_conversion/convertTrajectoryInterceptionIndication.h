/** ============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University
Copyright (c) 2024 Instituto de Telecomunicações, Universidade de Aveiro

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

#include <etsi_its_cpm_coding/cpm_TrajectoryInterceptionIndication.h>
#include <etsi_its_cpm_conversion/convertStationId.h>
#include <etsi_its_cpm_conversion/convertTrajectoryInterceptionConfidence.h>
#include <etsi_its_cpm_conversion/convertTrajectoryInterceptionProbability.h>
#ifdef ROS1
#include <etsi_its_cpm_msgs/TrajectoryInterceptionIndication.h>
namespace cpm_msgs = etsi_its_cpm_msgs;
#else
#include <etsi_its_cpm_msgs/msg/trajectory_interception_indication.hpp>
namespace cpm_msgs = etsi_its_cpm_msgs::msg;
#endif


namespace etsi_its_cpm_conversion {

void toRos_TrajectoryInterceptionIndication(const cpm_TrajectoryInterceptionIndication_t& in, cpm_msgs::TrajectoryInterceptionIndication& out) {
  if (in.subjectStation) {
    toRos_StationId(*in.subjectStation, out.subject_station);
    out.subject_station_is_present = true;
  }
  toRos_TrajectoryInterceptionProbability(in.trajectoryInterceptionProbability, out.trajectory_interception_probability);
  if (in.trajectoryInterceptionConfidence) {
    toRos_TrajectoryInterceptionConfidence(*in.trajectoryInterceptionConfidence, out.trajectory_interception_confidence);
    out.trajectory_interception_confidence_is_present = true;
  }
}

void toStruct_TrajectoryInterceptionIndication(const cpm_msgs::TrajectoryInterceptionIndication& in, cpm_TrajectoryInterceptionIndication_t& out) {
  memset(&out, 0, sizeof(cpm_TrajectoryInterceptionIndication_t));

  if (in.subject_station_is_present) {
    out.subjectStation = (cpm_StationId_t*) calloc(1, sizeof(cpm_StationId_t));
    toStruct_StationId(in.subject_station, *out.subjectStation);
  }
  toStruct_TrajectoryInterceptionProbability(in.trajectory_interception_probability, out.trajectoryInterceptionProbability);
  if (in.trajectory_interception_confidence_is_present) {
    out.trajectoryInterceptionConfidence = (cpm_TrajectoryInterceptionConfidence_t*) calloc(1, sizeof(cpm_TrajectoryInterceptionConfidence_t));
    toStruct_TrajectoryInterceptionConfidence(in.trajectory_interception_confidence, *out.trajectoryInterceptionConfidence);
  }
}

}
