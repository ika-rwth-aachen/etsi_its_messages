#include "displays/DENM/denm_render_object.hpp"
#include <variant>
#include <type_traits>
#include <cmath>

namespace etsi_its_msgs
{
namespace displays
{

  DENMRenderObject::DENMRenderObject(const std::variant<
      etsi_its_denm_msgs::msg::DENM,
      etsi_its_denm_ts_msgs::msg::DENM
    > & denm_variant)
  {
    int zone;
    bool northp;
    geometry_msgs::msg::PointStamped p;
    uint64_t nanosecs = 0;
    uint32_t station_id_val = 0;
    int station_type_val = 0;
    std::string cause_code_type_val = "Not present";
    std::string sub_cause_code_type_val = "Not present";
    double heading_deg = 0*M_PI/180.0; // 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
    double vehicle_speed = 0.0;

    if (const auto * r1 = std::get_if<etsi_its_denm_msgs::msg::DENM>(&denm_variant)) {
      is_ts = false;
      p = etsi_its_denm_msgs::access::getUTMPosition(*r1, zone, northp);
      nanosecs = etsi_its_denm_msgs::access::getUnixNanosecondsFromReferenceTime(etsi_its_denm_msgs::access::getReferenceTime(*r1));
      station_id_val = etsi_its_denm_msgs::access::getStationID(*r1);
      station_type_val = etsi_its_denm_msgs::access::getStationType(*r1);
      if(r1->denm.situation_is_present) {
        cause_code_type_val = etsi_its_denm_msgs::access::getCauseCodeType(*r1);
        sub_cause_code_type_val = etsi_its_denm_msgs::access::getSubCauseCodeType(*r1);
      }
      if (r1->denm.location_is_present && etsi_its_denm_msgs::access::getIsHeadingPresent(*r1)) {
        heading_deg = (90-etsi_its_denm_msgs::access::getHeading(*r1))*M_PI/180.0;
      }
      if (r1->denm.location_is_present && etsi_its_denm_msgs::access::getIsSpeedPresent(*r1)) {
        vehicle_speed = etsi_its_denm_msgs::access::getSpeed(*r1);
      }
    }
    else if (const auto * r2 = std::get_if<etsi_its_denm_ts_msgs::msg::DENM>(&denm_variant)) {
      is_ts = true;
      p = etsi_its_denm_ts_msgs::access::getUTMPosition(*r2, zone, northp);
      nanosecs = etsi_its_denm_ts_msgs::access::getUnixNanosecondsFromReferenceTime(etsi_its_denm_ts_msgs::access::getReferenceTime(*r2));
      station_id_val = etsi_its_denm_ts_msgs::access::getStationID(*r2);
      station_type_val = etsi_its_denm_ts_msgs::access::getStationType(*r2);
      if(r2->denm.situation_is_present) {
        cause_code_type_val = etsi_its_denm_ts_msgs::access::getCauseCodeType(*r2);
        sub_cause_code_type_val = etsi_its_denm_ts_msgs::access::getSubCauseCodeType(*r2);
      }
      if (r2->denm.location_is_present && etsi_its_denm_ts_msgs::access::getIsHeadingPresent(*r2)) {
        heading_deg = (90-etsi_its_denm_ts_msgs::access::getWGSHeading(*r2))*M_PI/180.0;
      }
      if (r2->denm.location_is_present && etsi_its_denm_ts_msgs::access::getIsSpeedPresent(*r2)) {
        vehicle_speed = etsi_its_denm_ts_msgs::access::getSpeed(*r2);
      }
    } else { // unexpected variant content (minimal safe initialization)
      is_ts = false;
      header.frame_id = "";
      header.stamp = rclcpp::Time(0);
      station_id = 0;
      station_type = 0;
      cause_code_type = "Not present";
      sub_cause_code_type = "Not present";
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
    cause_code_type = cause_code_type_val;
    sub_cause_code_type = sub_cause_code_type_val;
    double heading = heading_deg;
    while (heading < 0) heading += 2 * M_PI;
    pose.position = p.point;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading);
    pose.orientation = tf2::toMsg(orientation);
    speed = vehicle_speed;
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

  bool DENMRenderObject::isTS() {
    return is_ts;
  }
}  // namespace displays
}  // namespace etsi_its_msgs