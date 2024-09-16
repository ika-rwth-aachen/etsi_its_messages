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

#include "displays/CPM/cpm_render_object.hpp"

namespace etsi_its_msgs {
namespace displays {

CPMRenderObject::CPMRenderObject(etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage cpm, rclcpp::Time receive_time,
                                 uint16_t n_leap_seconds) {
  uint8_t number_of_objects = etsi_its_cpm_ts_msgs::access::getNumberOfPerceivedObjects(cpm);

  geometry_msgs::msg::Pose poses[number_of_objects];
  geometry_msgs::msg::Vector3 dimensionss[number_of_objects];
  geometry_msgs::msg::Vector3 velocities[number_of_objects];

  for (int i = 0; i < number_of_objects; i++) {
    etsi_its_cpm_ts_msgs::msg::PerceivedObject object = etsi_its_cpm_ts_msgs::access::getPerceivedObject(
        etsi_its_cpm_ts_msgs::access::getPerceivedObjectContainer(cpm), i);

    geometry_msgs::msg::Point position = etsi_its_cpm_ts_msgs::access::getPositionOfPerceivedObject(object);

    geometry_msgs::msg::Quaternion orientation = etsi_its_cpm_ts_msgs::access::getOrientationOfPerceivedObject(object);

    geometry_msgs::msg::Pose pose;

    pose.position = position;
    pose.orientation = orientation;
    poses[i] = pose;

    geometry_msgs::msg::Vector3 dimensions = etsi_its_cpm_ts_msgs::access::getDimensionsOfPerceivedObject(object);
    dimensions.z = 1.6;
    dimensionss[i] = dimensions;

    //header.frame_id = position.header.frame_id;
    header.frame_id = "map";

    // uint64_t nanosecs = etsi_its_cpm_ts_msgs::access::getUnixNanosecondsFromGenerationDeltaTime(etsi_its_cpm_ts_msgs::access::getGenerationDeltaTime(cpm), receive_time.nanoseconds(), n_leap_seconds);
    // header.stamp = rclcpp::Time(nanosecs);

    uint32_t station_id = etsi_its_cpm_ts_msgs::access::getStationID(cpm);

    geometry_msgs::msg::Vector3 velocity = etsi_its_cpm_ts_msgs::access::getCartesianVelocityOfPerceivedObject(object);
    velocities[i] = velocity;
  }
}

bool CPMRenderObject::validateFloats() {
  bool valid = true;
  valid = valid && rviz_common::validateFloats(pose);
  valid = valid && rviz_common::validateFloats(dimensions);
  valid = valid && rviz_common::validateFloats(velocity);
  return valid;
}

double CPMRenderObject::getAge(rclcpp::Time now) { return (now - header.stamp).seconds(); }

std_msgs::msg::Header CPMRenderObject::getHeader() { return header; }

uint32_t CPMRenderObject::getStationID() { return station_id; }

//   int CPMRenderObject::getStationType() {
//     return station_type;
//   }

geometry_msgs::msg::Pose CPMRenderObject::getPose(int i) { return poses[i]; }

int CPMRenderObject::getNumberOfObjects() { return number_of_objects; }

geometry_msgs::msg::Vector3 CPMRenderObject::getDimensions(int i) { return dimensionss[i]; }

geometry_msgs::msg::Vector3 CPMRenderObject::getSpeed(int i) { return velocities[i]; }

}  // namespace displays
}  // namespace etsi_its_msgs