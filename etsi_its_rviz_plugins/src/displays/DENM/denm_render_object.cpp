#include "displays/DENM/denm_render_object.hpp"

namespace etsi_its_msgs
{
namespace displays
{

  DENMRenderObject::DENMRenderObject(etsi_its_cam_msgs::msg::CAM cam, rclcpp::Time receive_time, uint16_t n_leap_seconds) {

    int zone;
    bool northp;
    geometry_msgs::msg::PointStamped p = etsi_its_cam_msgs::access::getUTMPosition(cam, zone, northp);
    header.frame_id = p.header.frame_id;
    
    uint64_t nanosecs = etsi_its_cam_msgs::access::getUnixNanosecondsFromGenerationDeltaTime(etsi_its_cam_msgs::access::getGenerationDeltaTime(cam), receive_time.nanoseconds(), n_leap_seconds);
    header.stamp = rclcpp::Time(nanosecs);

    station_id = etsi_its_cam_msgs::access::getStationID(cam);
    station_type = etsi_its_cam_msgs::access::getStationType(cam);
    
    double heading = (90-etsi_its_cam_msgs::access::getHeading(cam))*M_PI/180.0; // 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
    while(heading<0) heading+=2*M_PI;
    pose.position = p.point;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading);
    pose.orientation = tf2::toMsg(orientation);
    
    dimensions.x = etsi_its_cam_msgs::access::getVehicleLength(cam);
    dimensions.y = etsi_its_cam_msgs::access::getVehicleWidth(cam);
    dimensions.z = 1.6;

    speed = etsi_its_cam_msgs::access::getSpeed(cam);
  }

  bool DENMRenderObject::validateFloats() {
    bool valid = true;
    valid = valid && rviz_common::validateFloats(pose);
    valid = valid && rviz_common::validateFloats(dimensions);
    valid = valid && rviz_common::validateFloats(speed);
    return valid;
  }

  double DENMRenderObject::getAge(rclcpp::Time now) {
    return (now-header.stamp).seconds();
  }

  std_msgs::msg::Header DENMRenderObject::getHeader() {
    return header;
  }

  int DENMRenderObject::getStationID() {
    return station_id;
  }

  int DENMRenderObject::getStationType() {
    return station_type;
  }

  geometry_msgs::msg::Pose DENMRenderObject::getPose() {
    return pose;
  }

  geometry_msgs::msg::Vector3 DENMRenderObject::getDimensions() {

    return dimensions;
  }

  double DENMRenderObject::getSpeed() {
    return speed;
  }

}  // namespace displays
}  // namespace etsi_its_msgs