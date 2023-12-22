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

#include "displays/MAPEM/intersection_render_object.hpp"

namespace etsi_its_msgs
{
namespace displays
{

  IntersectionRenderObject::IntersectionRenderObject(etsi_its_mapem_msgs::msg::IntersectionGeometry intersection, bool timestamp_is_present, etsi_its_mapem_msgs::msg::MinuteOfTheYear mapem_stamp, rclcpp::Time receive_time) {

    intersection_id = etsi_its_msgs::J2735_access::getIntersectionID(intersection);

    int zone;
    bool northp;
    ref_point = etsi_its_msgs::J2735_access::getRefPointUTMPosition(intersection, zone, northp);

    if(timestamp_is_present) {
      uint64_t nanosecs = etsi_its_msgs::J2735_access::getUnixNanosecondsFromMinuteOfTheYear(mapem_stamp, receive_time.nanoseconds());
      header.stamp = rclcpp::Time(nanosecs);
    }
    else {
      header.stamp = receive_time;
    }
    header.frame_id = ref_point.header.frame_id;

  }

  bool IntersectionRenderObject::validateFloats() {
    bool valid = true;
    valid = valid && rviz_common::validateFloats(ref_point);
    return valid;
  }

  double IntersectionRenderObject::getAge(rclcpp::Time now) {
    return (now-header.stamp).seconds();
  }

  unsigned int IntersectionRenderObject::getIntersectionID() {
    return intersection_id;
  }

  std_msgs::msg::Header IntersectionRenderObject::getHeader() {
    return header;
  }

  geometry_msgs::msg::Point IntersectionRenderObject::getRefPosition() {
    return ref_point.point;
  }


}  // namespace displays
}  // namespace etsi_its_msgs