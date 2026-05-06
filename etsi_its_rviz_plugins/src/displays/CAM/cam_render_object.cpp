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

#include "displays/CAM/cam_render_object.hpp"
#include <variant>
#include <type_traits>
#include <cmath>

namespace etsi_its_msgs
{
namespace displays
{

  CAMRenderObject::CAMRenderObject(const std::variant<
      etsi_its_cam_msgs::msg::CAM,
      etsi_its_cam_ts_msgs::msg::CAM
    > & cam_variant, rclcpp::Time receive_time, uint16_t n_leap_seconds)
  {
    int zone;
    bool northp;
    geometry_msgs::msg::PointStamped p;
    uint64_t nanosecs = 0;
    uint32_t station_id_val = 0;
    int station_type_val = 0;
    double heading_deg = 0.0;
    double vehicle_length = 0.0;
    double vehicle_width = 0.0;
    double vehicle_speed = 0.0;

    if (const auto * r1 = std::get_if<etsi_its_cam_msgs::msg::CAM>(&cam_variant)) {
      is_ts = false;
      p = etsi_its_cam_msgs::access::getUTMPosition(*r1, zone, northp);
      nanosecs = etsi_its_cam_msgs::access::getUnixNanosecondsFromGenerationDeltaTime(etsi_its_cam_msgs::access::getGenerationDeltaTime(*r1), receive_time.nanoseconds(), n_leap_seconds);
      station_id_val = etsi_its_cam_msgs::access::getStationID(*r1);
      station_type_val = etsi_its_cam_msgs::access::getStationType(*r1);
      heading_deg = etsi_its_cam_msgs::access::getHeading(*r1);
      vehicle_length = etsi_its_cam_msgs::access::getVehicleLength(*r1);
      vehicle_width = etsi_its_cam_msgs::access::getVehicleWidth(*r1);
      vehicle_speed = etsi_its_cam_msgs::access::getSpeed(*r1);
    }
    else if (const auto * r2 = std::get_if<etsi_its_cam_ts_msgs::msg::CAM>(&cam_variant)) {
      is_ts = true;
      p = etsi_its_cam_ts_msgs::access::getUTMPosition(*r2, zone, northp);
      nanosecs = etsi_its_cam_ts_msgs::access::getUnixNanosecondsFromGenerationDeltaTime(etsi_its_cam_ts_msgs::access::getGenerationDeltaTime(*r2), receive_time.nanoseconds(), n_leap_seconds);
      station_id_val = etsi_its_cam_ts_msgs::access::getStationID(*r2);
      station_type_val = etsi_its_cam_ts_msgs::access::getStationType(*r2);
      heading_deg = etsi_its_cam_ts_msgs::access::getHeading(*r2);
      vehicle_length = etsi_its_cam_ts_msgs::access::getVehicleLength(*r2);
      vehicle_width = etsi_its_cam_ts_msgs::access::getVehicleWidth(*r2);
      vehicle_speed = etsi_its_cam_ts_msgs::access::getSpeed(*r2);
    } else { // unexpected variant content (minimal safe initialization)
      is_ts = false;
      header.frame_id = "";
      header.stamp = rclcpp::Time(0);
      station_id = 0;
      station_type = 0;
      pose = geometry_msgs::msg::Pose();
      dimensions = geometry_msgs::msg::Vector3();
      speed = 0.0;
      return;
    }

    // common initialization
    header.frame_id = p.header.frame_id;
    header.stamp = rclcpp::Time(nanosecs);
    station_id = station_id_val;
    station_type = station_type_val;
    double heading = (90.0 - heading_deg) * M_PI / 180.0;
    while (heading < 0) heading += 2 * M_PI;
    pose.position = p.point;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading);
    pose.orientation = tf2::toMsg(orientation);
    dimensions.x = vehicle_length;
    dimensions.y = vehicle_width;
    dimensions.z = 1.6;
    speed = vehicle_speed;
  }

  bool CAMRenderObject::validateFloats() {
    bool valid = true;
    valid = valid && rviz_common::validateFloats(pose);
    valid = valid && rviz_common::validateFloats(dimensions);
    valid = valid && rviz_common::validateFloats(speed);
    return valid;
  }

  double CAMRenderObject::getAge(rclcpp::Time now) {
    return (now-header.stamp).seconds();
  }

  std_msgs::msg::Header CAMRenderObject::getHeader() {
    return header;
  }

  uint32_t CAMRenderObject::getStationID() {
    return station_id;
  }

  int CAMRenderObject::getStationType() {
    return station_type;
  }

  geometry_msgs::msg::Pose CAMRenderObject::getPose() {
    return pose;
  }

  geometry_msgs::msg::Vector3 CAMRenderObject::getDimensions() {
    return dimensions;
  }

  double CAMRenderObject::getSpeed() {
    return speed;
  }

  bool CAMRenderObject::isTS() {
    return is_ts;
  }
}  // namespace displays
}  // namespace etsi_its_msgs