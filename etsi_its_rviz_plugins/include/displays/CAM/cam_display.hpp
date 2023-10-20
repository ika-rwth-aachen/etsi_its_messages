#pragma once

#include "etsi_its_cam_msgs/msg/cam.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/header.hpp"
#include <etsi_its_msgs/cam_access.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/objects/movable_text.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include <rclcpp/rclcpp.hpp>

namespace Ogre
{
class ManualObject;
}

namespace rviz_common
{
namespace properties
{
  class ColorProperty;
  class FloatProperty;
}  // namespace properties
}  // namespace rviz_common

namespace etsi_its_msgs
{
namespace displays
{

/**
 * @class CAMRenderObject
 * @brief 
 */
class CAMRenderObject
{
  public:
    CAMRenderObject(etsi_its_cam_msgs::msg::CAM cam, rclcpp::Time receive_time, uint16_t n_leap_seconds=5)
    {
      using namespace etsi_its_cam_msgs::access;
      int zone;
      bool northp;
      geometry_msgs::msg::PointStamped p = getUTMPosition(cam, zone, northp);
      header.frame_id = p.header.frame_id;
      uint64_t nanosecs = getUnixNanosecondsFromGenerationDeltaTime(getGenerationDeltaTime(cam), receive_time.nanoseconds(), n_leap_seconds);
      header.stamp = rclcpp::Time(nanosecs);
      // 0.0° equals WGS84 North, 90.0° equals WGS84 East, 180.0° equals WGS84 South and 270.0° equals WGS84 West
      double heading = (90-getHeading(cam))*M_PI/180.0;
      while(heading<0) heading+=2*M_PI;
      pose.position = p.point;
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, heading);
      pose.orientation = tf2::toMsg(orientation);
      station_id = getStationID(cam);
      width = getVehicleWidth(cam);
      length = getVehicleLength(cam);
      height = 1.6; // To-Do: Make sure, that there is definetly no Height within the CAM?!
      speed = getSpeed(cam);
    };
    ~CAMRenderObject(){};

    /**
     * @brief Get the age of a CAMRenderObject
     * 
     * @param now reference point in time to calculate the age with
     * @return age in seconds as double value 
     */
    double getAge(rclcpp::Time now)
    {
      return (now-header.stamp).seconds();
    };

    /**
     * @brief This function validates all float variables that are part of a CAMRenderObject
     * 
     */
    bool validateFloats()
    {
      bool valid = true;
      valid = valid && rviz_common::validateFloats(pose);
      valid = valid && rviz_common::validateFloats(width);
      valid = valid && rviz_common::validateFloats(length);
      valid = valid && rviz_common::validateFloats(height);
      valid = valid && rviz_common::validateFloats(speed);
      return valid;
    }

    // Public member variables
    std_msgs::msg::Header header;
    int station_id;
    double width, length, height;
    geometry_msgs::msg::Pose pose;
    double speed;
    
};

/**
 * @class CAMDisplay
 * @brief Displays an etsi_its_cam_msgs::CAM
 */
class CAMDisplay : public
  rviz_common::RosTopicDisplay<etsi_its_cam_msgs::msg::CAM>
{
  Q_OBJECT

public:
  CAMDisplay();
  ~CAMDisplay() override;

  void onInitialize() override;

  void reset() override;

protected:
  void processMessage(etsi_its_cam_msgs::msg::CAM::ConstSharedPtr msg) override;
  void update(float wall_dt, float ros_dt) override;

  Ogre::ManualObject * manual_object_;

  rclcpp::Node::SharedPtr rviz_node_;

  // Properties
  rviz_common::properties::BoolProperty *show_meta_, *show_station_id_, *show_speed_;
  rviz_common::properties::FloatProperty *buffer_timeout_, *bb_scale_, *char_height_;
  rviz_common::properties::ColorProperty *color_property_, *text_color_property_;

  std::vector<CAMRenderObject> cams_;
  std::vector<std::shared_ptr<rviz_rendering::Shape>> bboxs_;
  std::vector<std::shared_ptr<rviz_rendering::MovableText>> texts_;
};

}  // namespace displays
}  // namespace etsi_its_msgs