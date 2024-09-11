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

namespace etsi_its_msgs
{
namespace displays
{

  CPMRenderObject::CPMRenderObject(etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage cpm, rclcpp::Time receive_time, uint16_t n_leap_seconds) {

    // int zone;
    // bool northp;
    // geometry_msgs::msg::PointStamped p = etsi_its_cpm_ts_msgs::access::getUTMPosition(cpm, zone, northp);
    // header.frame_id = p.header.frame_id;

    // uint64_t nanosecs = etsi_its_cpm_ts_msgs::access::getUnixNanosecondsFromGenerationDeltaTime(etsi_its_cpm_msgs::access::getGenerationDeltaTime(cpm), receive_time.nanoseconds(), n_leap_seconds);
    // header.stamp = rclcpp::Time(nanosecs);

    // station_id = etsi_its_cpm_msgs::access::getStationID(cpm);
    // station_type = etsi_its_cpm_msgs::access::getStationType(cpm);

    // double heading = (90-etsi_its_cpm_msgs::access::getHeading(cpm))*M_PI/180.0; // 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
    // while(heading<0) heading+=2*M_PI;
    // pose.position = p.point;
    // tf2::Quaternion orientation;
    // orientation.setRPY(0.0, 0.0, heading);
    // pose.orientation = tf2::toMsg(orientation);

    // dimensions.x = etsi_its_cpm_msgs::access::getVehicleLength(cpm);
    // dimensions.y = etsi_its_cpm_msgs::access::getVehicleWidth(cpm);
    // dimensions.z = 1.6;

    // speed = etsi_its_cpm_msgs::access::getSpeed(cpm);
  }

  bool CPMRenderObject::validateFloats() {
    bool valid = true;
    valid = valid && rviz_common::validateFloats(pose);
    valid = valid && rviz_common::validateFloats(dimensions);
    valid = valid && rviz_common::validateFloats(speed);
    return valid;
  }

  double CPMRenderObject::getAge(rclcpp::Time now) {
    return (now-header.stamp).seconds();
  }

  std_msgs::msg::Header CPMRenderObject::getHeader() {
    return header;
  }

  uint32_t CPMRenderObject::getStationID() {
    return station_id;
  }

  int CPMRenderObject::getStationType() {
    return station_type;
  }

  geometry_msgs::msg::Pose CPMRenderObject::getPose() {
    return pose;
  }

  geometry_msgs::msg::Vector3 CPMRenderObject::getDimensions() {

    return dimensions;
  }

  double CPMRenderObject::getSpeed() {
    return speed;
  }

}  // namespace displays
}  // namespace etsi_its_msgs