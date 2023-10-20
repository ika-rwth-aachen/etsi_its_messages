#include "displays/CAM/cam_render_object.hpp"

namespace etsi_its_msgs
{
namespace displays
{

    CAMRenderObject::CAMRenderObject(etsi_its_cam_msgs::msg::CAM cam, rclcpp::Time receive_time, uint16_t n_leap_seconds)
    {
      using namespace etsi_its_cam_msgs::access;

      int zone;
      bool northp;
      geometry_msgs::msg::PointStamped p = getUTMPosition(cam, zone, northp);
      header.frame_id = p.header.frame_id;
      
      uint64_t nanosecs = getUnixNanosecondsFromGenerationDeltaTime(getGenerationDeltaTime(cam), receive_time.nanoseconds(), n_leap_seconds);
      header.stamp = rclcpp::Time(nanosecs);

      station_id = getStationID(cam);
      
      double heading = (90-getHeading(cam))*M_PI/180.0; // 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
      while(heading<0) heading+=2*M_PI;
      pose.position = p.point;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, heading);
      pose.orientation = tf2::toMsg(orientation);
      
      width = getVehicleWidth(cam);
      length = getVehicleLength(cam);
      height = 1.6;

      speed = getSpeed(cam);
    }

    double CAMRenderObject::getAge(rclcpp::Time now)
    {
      return (now-header.stamp).seconds();
    }

    bool CAMRenderObject::validateFloats()
    {
      bool valid = true;
      valid = valid && rviz_common::validateFloats(pose);
      valid = valid && rviz_common::validateFloats(width);
      valid = valid && rviz_common::validateFloats(length);
      valid = valid && rviz_common::validateFloats(height);
      valid = valid && rviz_common::validateFloats(speed);
      return valid;
    }

}  // namespace displays
}  // namespace etsi_its_msgs