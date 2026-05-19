// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

#pragma once

#include "etsi_its_cam_msgs/msg/cam.hpp"
#include "etsi_its_cam_ts_msgs/msg/cam.hpp"

#include "displays/CAM/cam_render_object.hpp"

#include "rviz_common/ros_topic_display.hpp"
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
 * @class CAMDisplay
 * @brief Displays an etsi_its_cam_msgs::CAM
 */
class CAMDisplay : public rviz_common::_RosTopicDisplay
{
  Q_OBJECT

public:
  CAMDisplay();
  ~CAMDisplay() override;

  void onInitialize() override;

  void reset() override;

  void setTopic(const QString & topic, const QString & datatype) override;

protected Q_SLOTS:
  void updateTopic() override;

protected:
  // CAM type detection
  enum class CamType
  {
    NONE,
    RELEASE_1,
    RELEASE_2
  };
  CamType detectCamType(const std::string & topic);

  void subscribe();
  void unsubscribe();
  void onEnable() override;
  void onDisable() override;

  // Unified handler that accepts either CAM (release 1) or CAM TS (release 2)
  void processMessage(const std::variant<
      etsi_its_cam_msgs::msg::CAM,
      etsi_its_cam_ts_msgs::msg::CAM
    > & msg_variant);

  void update(float wall_dt, float ros_dt) override;

  Ogre::ManualObject * manual_object_;

  rclcpp::Node::SharedPtr rviz_node_;
  std::shared_ptr<rclcpp::SubscriptionBase> subscription_;
  rclcpp::TimerBase::SharedPtr topic_check_timer_;

  CamType active_cam_type_;
  rclcpp::Time subscription_start_time_;
  uint32_t messages_received_;

  // Properties
  rviz_common::properties::BoolProperty *show_meta_, *show_station_id_, *show_speed_;
  rviz_common::properties::FloatProperty *buffer_timeout_, *bb_scale_, *char_height_;
  rviz_common::properties::ColorProperty *color_property_, *text_color_property_;

  std::unordered_map<int, CAMRenderObject> cams_;
  std::vector<std::shared_ptr<rviz_rendering::Shape>> bboxs_;
  std::vector<std::shared_ptr<rviz_rendering::MovableText>> texts_;
};

}  // namespace displays
}  // namespace etsi_its_msgs