#pragma once

#include "etsi_its_cam_msgs/msg/cam.hpp"
#include <etsi_its_msgs/cam_access.hpp>

#include "rviz_common/ros_topic_display.hpp"

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
 * \class CAMDisplay
 * \brief Displays an etsi_its_cam_msgs::CAM
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

  Ogre::ManualObject * manual_object_;

  // Properties
  // General
  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::FloatProperty *alpha_property_;
  rviz_common::properties::BoolProperty *viz_bounding_box_, *viz_direction_ind_, *viz_text_, *viz_velocity_, *viz_acceleration_;

  // Velocity Properties
  rviz_common::properties::FloatProperty *velocity_scale_;
  rviz_common::properties::BoolProperty *use_velocity_color_;
  rviz_common::properties::ColorProperty *velocity_color_property_;

  // Acceleration Properties
  rviz_common::properties::FloatProperty *acceleration_scale_;
  rviz_common::properties::BoolProperty *use_acceleration_color_;
  rviz_common::properties::ColorProperty *acceleration_color_property_;
  
  // Text Properties
  rviz_common::properties::FloatProperty *char_height_;
  rviz_common::properties::BoolProperty *print_vel_;

};

}  // namespace displays
}  // namespace etsi_its_msgs