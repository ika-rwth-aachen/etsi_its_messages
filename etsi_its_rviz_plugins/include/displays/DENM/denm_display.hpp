#pragma once

#include "etsi_its_denm_msgs/msg/denm.hpp"

#include "displays/DENM/denm_render_object.hpp"
#include "displays/DENM/overlay_object.hpp"

#include "rviz_common/ros_topic_display.hpp"
#include "rviz_rendering/objects/movable_text.hpp"
#include "rviz_rendering/objects/arrow.hpp"

#include <rclcpp/rclcpp.hpp>

namespace Ogre
{
class ManualObject;
}

namespace rviz_common
{
namespace properties
{
  class BoolProperty;
  class ColorProperty;
  class EnumProperty;
  class FloatProperty;
  class IntProperty;
}  // namespace properties
}  // namespace rviz_common

namespace etsi_its_msgs
{
namespace displays
{

/**
 * @class DENMDisplay
 * @brief Displays an etsi_its_denm_msgs::DENM
 */
class DENMDisplay : public
  rviz_common::RosTopicDisplay<etsi_its_denm_msgs::msg::DENM>
{
  Q_OBJECT

public:
  DENMDisplay();
  ~DENMDisplay() override;

  void onInitialize() override;

  void reset() override;

protected:
  void processMessage(etsi_its_denm_msgs::msg::DENM::ConstSharedPtr msg) override;
  void update(float wall_dt, float ros_dt) override;
  void updateOverlay(DENMRenderObject &denm_render_object);

private:
  Ogre::ManualObject * manual_object_;

  rclcpp::Node::SharedPtr rviz_node_;

  // Properties
  rviz_common::properties::BoolProperty *show_meta_, *show_station_id_, *show_cause_code_, *show_sub_cause_code_;
  rviz_common::properties::FloatProperty *buffer_timeout_, *char_height_;
  rviz_common::properties::ColorProperty *color_property_, *text_color_property_;

  std::unordered_map<int, DENMRenderObject> denms_;
  std::vector<std::shared_ptr<rviz_rendering::Arrow>> arrows_;
  std::vector<std::shared_ptr<rviz_rendering::MovableText>> texts_;

  // overlay
  rviz_common::properties::BoolProperty *show_overlay_prop_;
  rviz_common::properties::IntProperty *top_offset_prop_, *left_offset_prop_, *text_size_prop_, *line_width_prop_;
  rviz_common::properties::ColorProperty *bg_color_prop_, *fg_color_prop_;
  rviz_common::properties::FloatProperty *bg_alpha_prop_, *fg_alpha_prop_;
  rviz_common::properties::EnumProperty *font_prop_;

  std::shared_ptr<rviz_plugin::OverlayObject> overlay_;
  QStringList font_families_;
};

}  // namespace displays
}  // namespace etsi_its_msgs