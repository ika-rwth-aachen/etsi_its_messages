/** ============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

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

CPMRenderObject::CPMRenderObject(const etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage cpm) {

  station_id_ = etsi_its_cpm_ts_msgs::access::getStationID(cpm);
  reference_position_ = etsi_its_cpm_ts_msgs::access::getUTMPosition(cpm);

  uint64_t nanosecs = etsi_its_cpm_ts_msgs::access::getUnixNanosecondsFromReferenceTime(etsi_its_cpm_ts_msgs::access::getReferenceTime(cpm));
  header_.stamp = rclcpp::Time(nanosecs);
  header_.frame_id = reference_position_.header.frame_id;

  int n_perceived_objects = etsi_its_cpm_ts_msgs::access::getNumberOfPerceivedObjects(cpm);
  for (int i = 0; i < n_perceived_objects; i++) {
    etsi_its_cpm_ts_msgs::msg::PerceivedObject perceived_obj = etsi_its_cpm_ts_msgs::access::getPerceivedObject(
      etsi_its_cpm_ts_msgs::access::getPerceivedObjectContainer(cpm), i);
    geometry_msgs::msg::PointStamped utm_position = etsi_its_cpm_ts_msgs::access::getUTMPositionOfPerceivedObject(cpm, perceived_obj);
    
    Object obj;
    obj.pose.position = utm_position.point;
    if(perceived_obj.angles_is_present) obj.pose.orientation = etsi_its_cpm_ts_msgs::access::getOrientationOfPerceivedObject(perceived_obj);
    if(perceived_obj.object_dimension_x_is_present && perceived_obj.object_dimension_y_is_present && perceived_obj.object_dimension_z_is_present) {
      obj.dimensions = etsi_its_cpm_ts_msgs::access::getDimensionsOfPerceivedObject(perceived_obj);
    }
    if(perceived_obj.velocity_is_present) obj.velocity = etsi_its_cpm_ts_msgs::access::getCartesianVelocityOfPerceivedObject(perceived_obj);
    objects_.push_back(obj);
  }
}

bool CPMRenderObject::validateFloats() {
  bool valid = true;
  for (size_t i = 0; i < objects_.size(); i++) {
    valid = valid && rviz_common::validateFloats(objects_[i].pose);
    valid = valid && rviz_common::validateFloats(objects_[i].dimensions);
    valid = valid && rviz_common::validateFloats(objects_[i].velocity);
  }
  return valid;
}

double CPMRenderObject::getAge(const rclcpp::Time now) { return (now - header_.stamp).seconds(); }

std_msgs::msg::Header CPMRenderObject::getHeader() { return header_; }

uint32_t CPMRenderObject::getStationID() { return station_id_; }

geometry_msgs::msg::PointStamped CPMRenderObject::getReferencePosition() { return reference_position_; }

geometry_msgs::msg::Pose CPMRenderObject::getPoseOfObject(const uint8_t idx) { return objects_[idx].pose; }

geometry_msgs::msg::Vector3 CPMRenderObject::getDimensionsOfObject(const uint8_t idx) { return objects_[idx].dimensions; }

geometry_msgs::msg::Vector3 CPMRenderObject::getVelocityOfObject(const uint8_t idx) { return objects_[idx].velocity; }

uint8_t CPMRenderObject::getNumberOfObjects() { return objects_.size(); }

}  // namespace displays
}  // namespace etsi_its_msgs