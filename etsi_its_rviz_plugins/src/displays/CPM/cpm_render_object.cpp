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
                                 uint16_t n_leap_seconds, uint8_t number_of_object) {
                                
    etsi_its_cpm_ts_msgs::msg::PerceivedObject object = etsi_its_cpm_ts_msgs::access::getPerceivedObject(
        etsi_its_cpm_ts_msgs::access::getPerceivedObjectContainer(cpm), number_of_object);

    geometry_msgs::msg::PointStamped utm_position = etsi_its_cpm_ts_msgs::access::getUTMPositionOfPerceivedObject(cpm, object);

    pose_.position = utm_position.point;
    pose_.orientation = etsi_its_cpm_ts_msgs::access::getOrientationOfPerceivedObject(object);
    dimensions_ = etsi_its_cpm_ts_msgs::access::getDimensionsOfPerceivedObject(object);
    velocity_ = etsi_its_cpm_ts_msgs::access::getCartesianVelocityOfPerceivedObject(object);

    header_.frame_id =  utm_position.header.frame_id;
    uint64_t nanosecs = etsi_its_cpm_ts_msgs::access::getUnixNanosecondsFromReferenceTime(etsi_its_cpm_ts_msgs::access::getReferenceTime(cpm));
    header_.stamp = rclcpp::Time(nanosecs);

    station_id_ = etsi_its_cpm_ts_msgs::access::getStationID(cpm);
}

bool CPMRenderObject::validateFloats() {
  bool valid = true;
  valid = valid && rviz_common::validateFloats(pose_);
  valid = valid && rviz_common::validateFloats(dimensions_);
  valid = valid && rviz_common::validateFloats(velocity_);
  return valid;
}

double CPMRenderObject::getAge(rclcpp::Time now) { return (now - header_.stamp).seconds(); }

std_msgs::msg::Header CPMRenderObject::getHeader() { return header_; }

uint32_t CPMRenderObject::getStationID() { return station_id_; }

geometry_msgs::msg::Pose CPMRenderObject::getPose() { return pose_; }

geometry_msgs::msg::Vector3 CPMRenderObject::getDimensions() { return dimensions_; }

geometry_msgs::msg::Vector3 CPMRenderObject::getVelocity() { return velocity_; }

}  // namespace displays
}  // namespace etsi_its_msgs