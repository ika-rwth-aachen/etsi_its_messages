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

    //for getAge()
    etsi_its_denm_msgs::msg::TimestampIts t_its;
    etsi_its_denm_msgs::msg::TimestampIts timestamp_estimate = denm.denm.management.detection_time;
    Ogre::uint64 generation_delta_time_value = denm.denm.management.reference_time.value - denm.denm.management.detection_time.value;
    t_its.value = std::floor(timestamp_estimate.value/65536)*65536+generation_delta_time_value;
    cdd::throwIfOutOfRange(t_its.value, etsi_its_denm_msgs::msg::TimestampIts::MIN, etsi_its_denm_msgs::msg::TimestampIts::MAX, "TimestampIts");
    etsi_its_denm_msgs::msg::TimestampIts time_its = t_its;
    uint64_t nanosecs = time_its.value*1e6+etsi_its_msgs::UNIX_SECONDS_2004*1e9-n_leap_seconds*1e9;
    header.stamp = rclcpp::Time(nanosecs);
    
    //getStationID()
    station_id = denm.header.station_id.value;
    station_type = 5; //station_type = etsi_its_denm_msgs::access::getStationType(denm);
    //getCauseCode()
    cause_code = denm.denm.situation.event_type.cause_code.value;
    if(cause_code == 1) cause_code_type = "traffic condition";
    else if(cause_code == 2) cause_code_type = "accident";
    else if(cause_code == 3) cause_code_type = "roadworks";
    else if(cause_code == 5) cause_code_type = "impassibility";
    else if(cause_code == 6) cause_code_type = "adverse weather condition - adhesion";
    else if(cause_code == 7) cause_code_type = "aquaplanning";
    else if(cause_code == 9) cause_code_type = "hazardous location - surface condition";
    else if(cause_code == 10) cause_code_type = "hazardous location - obstacle on the road";
    else if(cause_code == 11) cause_code_type = "hazardous location - animal on the road";
    else if(cause_code == 12) cause_code_type = "human presence on the road";
    else if(cause_code == 14) cause_code_type = "wrong way driving";
    else if(cause_code == 15) cause_code_type = "rescue and recovery in progress";
    else if(cause_code == 17) cause_code_type = "adverse weather condition - extreme weather condition";
    else if(cause_code == 18) cause_code_type = "adverse weather condition - visibility";
    else if(cause_code == 19) cause_code_type = "adverse weather condition - precipitation";
    else if(cause_code == 26) cause_code_type = "slow vehicle";
    else if(cause_code == 27) cause_code_type = "dangerous end of queue";
    else if(cause_code == 91) cause_code_type = "vehicle breakdown";
    else if(cause_code == 92) cause_code_type = "post crash";
    else if(cause_code == 93) cause_code_type = "human problem";
    else if(cause_code == 94) cause_code_type = "stationary vehicle";
    else if(cause_code == 95) cause_code_type = "emergency vehicle approaching";
    else if(cause_code == 96) cause_code_type = "hazardous location - dangerous curve";
    else if(cause_code == 97) cause_code_type = "collision risk";
    else if(cause_code == 98) cause_code_type = "signal violation";
    else if(cause_code == 99) cause_code_type = "dangerous situation";
    
    //getHeading
    double heading; // 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
    if(denm.denm.location.event_position_heading_is_present){
      heading = (90-denm.denm.location.event_position_heading.heading_value.value)*M_PI/180.0;
    }
    else{
      heading = 0*M_PI/180.0;
    }
    while(heading<0) heading+=2*M_PI;
    pose.position = p.point;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading);
    pose.orientation = tf2::toMsg(orientation);
    
    dimensions.x = 1;
    dimensions.y = 1;
    dimensions.z = 1.6;

    //getSpeed()
    if(denm.denm.location.event_speed_is_present){
      speed = denm.denm.location.event_speed.speed_value.value * 0.01;
    }
    else{
      speed = 0;
    }
  }

  bool DENMRenderObject::validateFloats() {
    bool valid = true;
    valid = valid && rviz_common::validateFloats(pose);
    valid = valid && rviz_common::validateFloats(dimensions);
    valid = valid && rviz_common::validateFloats(speed);
    return valid;
  }

  double DENMRenderObject::getAge(rclcpp::Time now) {
    return (now-header.stamp).seconds()*0.015;
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

  std::string DENMRenderObject::getCauseCode(){
    return cause_code_type;
  }

}  // namespace displays
}  // namespace etsi_its_msgs