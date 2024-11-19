#include "displays/DENM/denm_display.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/bool_property.hpp"

#include "rviz_common/properties/parse_color.hpp"

namespace etsi_its_msgs
{
namespace displays
{

DENMDisplay::DENMDisplay()
{
  // General Properties
  buffer_timeout_ = new rviz_common::properties::FloatProperty(
    "Timeout", 0.1f,
    "Time (in s) until visualizations disappear", this);
  buffer_timeout_->setMin(0);
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 0, 25),
    "Color", this);
  show_meta_ = new rviz_common::properties::BoolProperty("Metadata", true, 
    "Show metadata as text next to objects", this);
  text_color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 0, 25),
    "Text color", show_meta_);
  char_height_ = new rviz_common::properties::FloatProperty("Scale", 4.0, "Scale of text", show_meta_);
  show_station_id_ = new rviz_common::properties::BoolProperty("StationID", true, 
    "Show StationID", show_meta_);
  show_cause_code_ = new rviz_common::properties::BoolProperty("CauseCode", true, "Show CauseCode", show_meta_);
  show_sub_cause_code_ = new rviz_common::properties::BoolProperty("SubCauseCode", true, "Show SubCauseCode", show_meta_);
}

DENMDisplay::~DENMDisplay()
{
  if (initialized() ) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void DENMDisplay::onInitialize()
{
  RTDClass::onInitialize();

  auto nodeAbstraction = context_->getRosNodeAbstraction().lock();
  rviz_node_ = nodeAbstraction->get_raw_node();

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
}

void DENMDisplay::reset()
{
  RTDClass::reset();
  manual_object_->clear();
}

void DENMDisplay::processMessage(etsi_its_denm_msgs::msg::DENM::ConstSharedPtr msg)
{
  // Generate DENM render object from message
  rclcpp::Time now = rviz_node_->now();
  DENMRenderObject denm(*msg);
  if (!denm.validateFloats()) {
        setStatus(
          rviz_common::properties::StatusProperty::Error, "Topic",
          "Message contained invalid floating point values (nans or infs)");
        return;
  }
  
  // Check if Station ID is already present in list
  auto it = denms_.find(denm.getStationID());
  if (it != denms_.end()) it->second = denm; // Key exists, update the value
  else denms_.insert(std::make_pair(denm.getStationID(), denm)); 
  
  return;
}

void DENMDisplay::update(float, float)
{
  
  // Check for outdated DENMs
  for (auto it = denms_.begin(); it != denms_.end(); ) {
        if (it->second.getAge(rviz_node_->now()) > buffer_timeout_->getFloat()) it = denms_.erase(it);
        else ++it;
  }
  
  // Render all valid denms
  arrows_.clear();
  texts_.clear();
  for(auto it = denms_.begin(); it != denms_.end(); ++it) {

    DENMRenderObject denm = it->second;
    Ogre::Vector3 sn_position;
    Ogre::Quaternion sn_orientation;
    if (!context_->getFrameManager()->getTransform(denm.getHeader(), sn_position, sn_orientation)) {
      setMissingTransformToFixedFrame(denm.getHeader().frame_id);
      return;
    }
    setTransformOk();

    // set pose of scene node
    scene_node_->setPosition(sn_position);
    scene_node_->setOrientation(sn_orientation);

    auto child_scene_node = scene_node_->createChildSceneNode();
    // Set position of scene node
    geometry_msgs::msg::Pose pose = denm.getPose();
    Ogre::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
    Ogre::Quaternion orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    //set size parameters for arrow
    int shaft_length = 5;
    int shaft_diameter = 1;
    int head_length = 2;
    int head_diameter = 3;
    position.z += (shaft_length+head_length);
    
    // set pose of child scene node arrow
    child_scene_node->setPosition(position);
    child_scene_node->setOrientation(orientation);
    
    // create arrow object
    std::shared_ptr<rviz_rendering::Arrow> arrow = std::make_shared<rviz_rendering::Arrow>(scene_manager_, child_scene_node, shaft_length, shaft_diameter, head_length, head_diameter);
    
    // set the color of arrow
    Ogre::ColourValue bb_color = rviz_common::properties::qtToOgre(color_property_->getColor());
    arrow->setColor(bb_color);
    arrow->setOrientation(orientation);
    arrows_.push_back(arrow);

    // Visualize meta-information as text
    if(show_meta_->getBool()) {
      std::string text;
      if(show_station_id_->getBool()) {
        text+="StationID: " + std::to_string(denm.getStationID());
        text+="\n";
      }
      if(show_cause_code_->getBool()) {
        text+="Cause: " + denm.getCauseCode();
        text+="\n";
      }
      if(show_sub_cause_code_->getBool()) {
        text+="SubCause: " + denm.getSubCauseCode();
      }
   
      if(!text.size()) return;
      std::shared_ptr<rviz_rendering::MovableText> text_render = std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", char_height_->getFloat());
      double height = text_render->getBoundingRadius();
      Ogre::Vector3 offs(0.0, 0.0, height);
      // There is a bug in rviz_rendering::MovableText::setGlobalTranslation https://github.com/ros2/rviz/issues/974
      text_render->setGlobalTranslation(offs);
      Ogre::ColourValue text_color = rviz_common::properties::qtToOgre(text_color_property_->getColor());
      text_render->setColor(text_color);
      child_scene_node->attachObject(text_render.get());
      texts_.push_back(text_render);
    }
  }
}

}  // namespace displays
}  // namespace etsi_its_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(etsi_its_msgs::displays::DENMDisplay, rviz_common::Display)