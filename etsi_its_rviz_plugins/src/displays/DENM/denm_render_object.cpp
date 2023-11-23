#include "displays/DENM/denm_render_object.hpp"

namespace etsi_its_msgs
{
namespace displays
{

  DENMRenderObject::DENMRenderObject(etsi_its_denm_msgs::msg::DENM denm, rclcpp::Time receive_time, uint16_t n_leap_seconds) {
    //getUTMPosition()
    int zone;
    bool northp;
    geometry_msgs::msg::PointStamped p;
    double latitude = (denm.denm.management.event_position.latitude.value)*1e-7;
    double longitude = (denm.denm.management.event_position.longitude.value)*1e-7;
    p.point.z = (denm.denm.management.event_position.altitude.altitude_value.value)*1e-2;
    try {
      GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, p.point.x, p.point.y);
      std::string hemisphere;
      if(northp) hemisphere="N";
      else hemisphere="S";
      p.header.frame_id="utm_"+std::to_string(zone)+hemisphere;
    } catch (GeographicLib::GeographicErr& e) {
      throw std::invalid_argument(e.what());
    }
    header.frame_id = p.header.frame_id;
    
    uint64_t nanosecs = etsi_its_denm_msgs::access::getUnixNanosecondsFromGenerationDeltaTime(etsi_its_denm_msgs::access::getGenerationDeltaTime(denm), receive_time.nanoseconds(), n_leap_seconds);
        header.stamp = rclcpp::Time(nanosecs);

    station_id = etsi_its_denm_msgs::access::getStationID(denm);
    station_type = etsi_its_denm_msgs::access::getStationType(denm);
    
        double heading = (90-etsi_its_denm_msgs::access::getHeading(denm))*M_PI/180.0; // 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
    while(heading<0) heading+=2*M_PI;
    pose.position = p.point;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading);
    pose.orientation = tf2::toMsg(orientation);
    
    dimensions.x = etsi_its_denm_msgs::access::getVehicleLength(denm);
    dimensions.y = etsi_its_denm_msgs::access::getVehicleWidth(denm);
        dimensions.z = 1.6;

    speed = etsi_its_denm_msgs::access::getSpeed(denm);
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