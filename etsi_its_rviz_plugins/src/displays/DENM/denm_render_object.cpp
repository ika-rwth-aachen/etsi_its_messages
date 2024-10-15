#include "displays/DENM/denm_render_object.hpp"

namespace etsi_its_msgs
{
namespace displays
{

  DENMRenderObject::DENMRenderObject(etsi_its_denm_msgs::msg::DENM denm) {

    int zone;
    bool northp;
    geometry_msgs::msg::PointStamped p = etsi_its_denm_msgs::access::getUTMPosition(denm, zone, northp);
    header.frame_id = p.header.frame_id;
  
    uint64_t nanosecs = etsi_its_denm_msgs::access::getUnixNanosecondsFromReferenceTime(etsi_its_denm_msgs::access::getReferenceTime(denm));
    header.stamp = rclcpp::Time(nanosecs);

    station_id = etsi_its_denm_msgs::access::getStationID(denm);
    cause_code_type = etsi_its_denm_msgs::access::getCauseCodeType(denm);
    sub_cause_code_type = etsi_its_denm_msgs::access::getSubCauseCodeType(denm);
    
    double heading; // 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
    if(etsi_its_denm_msgs::access::getIsHeadingPresent(denm)) {
      heading = (90-etsi_its_denm_msgs::access::getHeading(denm))*M_PI/180.0;
    }
    else {
      heading = 0*M_PI/180.0;
    }
    while(heading<0) heading+=2*M_PI;
    pose.position = p.point;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading);
    pose.orientation = tf2::toMsg(orientation);

    if(etsi_its_denm_msgs::access::getIsSpeedPresent(denm)) {
      speed = etsi_its_denm_msgs::access::getSpeed(denm);
    }
    else {
      speed = 0;
    }
  }

  bool DENMRenderObject::validateFloats() {
    bool valid = true;
    valid = valid && rviz_common::validateFloats(pose);
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

  geometry_msgs::msg::Pose DENMRenderObject::getPose() {
    return pose;
  }

  double DENMRenderObject::getSpeed() {
  return speed;
  }

  std::string DENMRenderObject::getCauseCode() {
    return cause_code_type;
  }

  std::string DENMRenderObject::getSubCauseCode() {
    return sub_cause_code_type;
  }

}  // namespace displays
}  // namespace etsi_its_msgs